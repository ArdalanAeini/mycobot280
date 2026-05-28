"""Top-down vision pick-and-place for a narrow cup (35 mm diameter).

Two-tier altitude design:
  - SAFE TRANSIT (default 150 mm above table): used for ALL long horizontal
    moves. The arm always rises to this altitude before traveling sideways
    to a new XY, then drops vertically.
  - HOVER (default 100 mm above table): the working altitude RIGHT BEFORE
    descending to grasp/release. Pure vertical descents happen below this.

Trajectory:
  0.   HOME            joint return to [0,0,0,0,0,0]
  0.5  SAFE_PICK       LONG horizontal move at safe altitude above cup XY
  1.   HOVER_PICK      pure vertical descent to hover altitude
  2.   DESCEND_PICK    pure vertical descent to grasp altitude
  3.   CLOSE           grip the cup
  4.   LIFT_PICK       pure vertical lift back to hover
  4.5  SAFE_PICK       pure vertical lift back to safe altitude
  5.   SAFE_PLACE      LONG horizontal move at safe altitude to place XY
  5b.  HOVER_PLACE     pure vertical descent to hover above place
  6.   DESCEND_PLACE   pure vertical descent to release altitude
  7.   OPEN            release the cup
  8.   LIFT_PLACE      pure vertical lift back to hover
  8b.  SAFE_PLACE      pure vertical lift back to safe altitude
  9.   HOME            joint return to [0,0,0,0,0,0]

The gripper points straight down (RPY = 180, 0, 0) for every move.
The AG gripper opens to 45 mm, which straddles the 35 mm cup with
5 mm clearance on each side.

Why two altitudes:
  The MyCobot 280 uses joint-interpolated Cartesian moves by default. For
  long horizontal moves, joint interpolation traces a CURVED path that can
  dip lower than both endpoints. Without safe-transit, the curve can drag
  the gripper into or through the cup during the approach. Splitting the
  motion into 'go HIGH, then drop' keeps every long horizontal move at the
  safe altitude where no collision is possible.

Design:
  - RPY is always (180, 0, 0).
  - Cup height hardcoded to DEFAULT_CUP_HEIGHT_MM (51 mm); camera gives XY only.
  - No pre-pose flip, no J6 rotation, no teach constants.
  - Self-contained: all helpers inlined, no imports from goto_cup.

Usage:
    python pick_place_cup_topdown.py --port /dev/ttyUSB0
    python pick_place_cup_topdown.py --dry-run
    python pick_place_cup_topdown.py --grasp-height-mm 30 --hover-mm 80
    python pick_place_cup_topdown.py --place-offset-mm 200 -300
    python pick_place_cup_topdown.py --safe-transit-mm 180
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import Sequence

import numpy as np

from locate_cup import (
    DEFAULT_MIN_CONFIDENCE,
    DEFAULT_PROFILE,
    DEFAULT_SETTLE_FRAMES,
    DEFAULT_TIMEOUT_S,
    CupNotFoundError,
    locate_cup_robot_mm,
    with_height_override,
)
from perception.calibration import CalibrationProfileIO
from perception.control import (
    Gripper,
    GripperSettings,
    MotionContext,
    MotionSettings,
    MyCobotDriver,
    MyCobotDriverSettings,
    ReachabilityError,
    move_to_world,
    project_above_table,
    world_to_robot,
)

# =============================================================================
# The only orientation we ever use.
# Gripper points straight down. Always.
# =============================================================================
TOP_DOWN_RPY = (180.0, 0.0, 0.0)

# =============================================================================
# Geometry
# =============================================================================
DEFAULT_CUP_HEIGHT_MM   = 51.0   # known physical height; camera gives XY only
DEFAULT_HOVER_MM        = 100.0  # working hover (above cup, before descent)
DEFAULT_SAFE_TRANSIT_MM = 150.0  # safe altitude for long horizontal moves
                                  # (always above hover; arm travels HIGH then descends)
DEFAULT_GRASP_HEIGHT_MM = 35.0   # fingers at mid-body of cup
DEFAULT_RELEASE_MM      = 35.0   # release height = same as grasp height
DEFAULT_GRASP_CLOSE_VALUE = 80   # 0=open, 100=closed
DEFAULT_PLACE_OFFSET_MM: tuple[float, float] = (100.0, -150.0)

# Calibrated XY shift applied to the PICK target only (not place).
# Two modes:
#   1. CLI override: pass --grasp-offset-mm DX DY with non-zero values
#      and it wins over everything else.
#   2. Auto lookup: when CLI flag is (0, 0), the script uses the lookup
#      table below to pick a Y-offset based on the cup's world-Y position.
#      The X offset is always DEFAULT_GRASP_X_OFFSET_MM.
DEFAULT_GRASP_OFFSET_MM: tuple[float, float] = (0.0, 0.0)

# Lookup table: cup world-Y position (mm) -> (X grasp offset, Y grasp offset).
# Bins are 20 mm wide (2 cm intervals). EACH bin carries its own X and Y
# offset so you can tune them independently per line.
#
# X is set to 4.0 mm everywhere for now -- edit any individual line's
# middle value to change the X offset for that bin only.
#
# To tune a bin: edit that single line. The lookup picks the nearest bin
# to the detected cup_world_y; cups outside the table range clamp to the
# nearest end.
#
# Format: (cup_world_y_center_mm, x_grasp_offset_mm, y_grasp_offset_mm)
GRASP_LOOKUP_TABLE: list[tuple[float, float, float]] = [
    (-200.0, 4.0,  20.0),
    (-180.0, 4.0,  20.0),
    (-160.0, 4.0,  20.0),
    (-140.0, 4.0,  -40.0),
    (-120.0, 4.0,  -10),
    (-100.0, 4.0,  -15.0),
    ( -80.0, 4.0,  10.0),
    ( -60.0, 4.0,  15),
    ( -40.0, 2.0,  20),
    ( -20.0, 4.0,  0),
    (   0.0, 4.0,   0.0),
    (  20.0, 4.0,   -5),
    (  40.0, 4.0,   10),
    (  60.0, 4.0,   -5),
    (  80.0, 4.0,   -5),
    ( 100.0, 4.0,   -5),
    ( 120.0, 4.0,   -5),
    ( 140.0, 4.0,   -5),
    ( 160.0, 4.0,   -5),
    ( 180.0, 4.0,   -5),
    ( 200.0, 4.0,   -5),
]


def _lookup_grasp_offset_mm(
    cup_world_y_mm: float,
) -> tuple[float, float, str]:
    """Pick the closest bin in GRASP_LOOKUP_TABLE for cup_world_y_mm.

    Returns (x_offset_mm, y_offset_mm, debug_string). Clamps to the
    nearest end if the cup is outside the table range.
    """
    closest = min(
        GRASP_LOOKUP_TABLE,
        key=lambda row: abs(row[0] - cup_world_y_mm),
    )
    bin_center, x_off, y_off = closest
    dbg = (
        f"bin Y={bin_center:+.0f} mm -> "
        f"x_offset={x_off:+.1f} mm, y_offset={y_off:+.1f} mm"
    )
    return x_off, y_off, dbg
REACH_MARGIN_MM         = 5.0
MIN_HOVER_OVER_GRASP_MM = 8.0

# =============================================================================
# Home joints (simple, no J6 flip needed for top-down)
# =============================================================================
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
JOINT_SETTLE_TOLERANCE_DEG = 3.0
JOINT_SETTLE_TIMEOUT_S     = 20.0


# =============================================================================
# Joint helpers
# =============================================================================
def _joint_error_deg(cur: float, tgt: float, idx: int) -> float:
    if idx == 5:
        d = (float(tgt) - float(cur)) % 360.0
        if d > 180.0:
            d -= 360.0
        return abs(d)
    return abs(float(cur) - float(tgt))


def _max_joint_error(cur: Sequence[float], tgt: Sequence[float]) -> float:
    return max(_joint_error_deg(cur[i], tgt[i], i) for i in range(6))


def _move_joints_and_wait(
    driver: MyCobotDriver,
    label: str,
    target: Sequence[float],
    *,
    speed: int = 30,
    timeout_s: float = JOINT_SETTLE_TIMEOUT_S,
) -> bool:
    target = [float(v) for v in target]
    print(f"[pick_top] moving to {label}: {target}")
    try:
        driver.send_angles_deg(target, speed=int(speed))
    except Exception as exc:
        print(f"[pick_top] WARN: send_angles failed for {label}: {exc}")
        return False
    deadline = time.monotonic() + float(timeout_s)
    last_err  = float("inf")
    while time.monotonic() < deadline:
        try:
            cur = driver.get_angles_deg(retries=2)
        except Exception:
            time.sleep(0.1)
            continue
        last_err = _max_joint_error(cur, target)
        if last_err <= JOINT_SETTLE_TOLERANCE_DEG:
            print(f"[pick_top] settled at {label} (err {last_err:.1f} deg)")
            return True
        time.sleep(0.15)
    print(f"[pick_top] WARN: timed out at {label} (err {last_err:.1f} deg)")
    return False


# =============================================================================
# MotionContext builder
# =============================================================================
def _build_context(
    profile_path: str, *, speed: int, max_reach_mm: float
) -> tuple[MotionContext, np.ndarray]:
    profile = CalibrationProfileIO.load(profile_path)
    t_robot_world = profile.get_robot_world_transform()
    if t_robot_world is None or profile.table_plane is None:
        raise ValueError(
            f"profile {profile_path!r} missing robot_world_transform "
            "or table_plane; re-run touch calibration."
        )
    n_world, o_world = profile.table_plane.as_arrays()
    settings = MotionSettings(
        max_reach_m=float(max_reach_mm) * 1e-3,
        default_speed=int(speed),
        vertical_rpy_deg=TOP_DOWN_RPY,
    )
    ctx = MotionContext(
        t_robot_world=t_robot_world,
        table_normal_world=n_world,
        table_origin_world=o_world,
        settings=settings,
    )
    return ctx, np.asarray(t_robot_world, dtype=np.float64)


# =============================================================================
# Reach helpers (firmware handles actual reachability)
# =============================================================================
def _rpy_to_rot(rpy_deg) -> np.ndarray:
    rx, ry, rz = np.radians(np.asarray(rpy_deg, dtype=np.float64))
    cx, sx = float(np.cos(rx)), float(np.sin(rx))
    cy, sy = float(np.cos(ry)), float(np.sin(ry))
    cz, sz = float(np.cos(rz)), float(np.sin(rz))
    return (
        np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]], dtype=np.float64)
        @ np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]], dtype=np.float64)
        @ np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]], dtype=np.float64)
    )


def _reach_ok(p_world_m: np.ndarray, ctx) -> tuple[bool, float]:
    return True, 0.0


def _check_reach(label: str, p_world_m: np.ndarray, ctx) -> None:
    pass


def _try_place_anchor(
    cup_world_m: np.ndarray,
    place_offset_mm: tuple[float, float],
    *,
    hover_h_m: float,
    release_h_m: float,
    ctx,
) -> tuple[np.ndarray, tuple[float, float], float] | None:
    ox, oy = float(place_offset_mm[0]), float(place_offset_mm[1])
    for scale in (1.0, 0.85, 0.7, 0.55, 0.4, 0.25, 0.0):
        dx, dy = ox * scale, oy * scale
        anchor = cup_world_m + np.array([dx*1e-3, dy*1e-3, 0.0])
        if (
            _reach_ok(project_above_table(anchor, hover_h_m,   ctx), ctx)[0]
            and
            _reach_ok(project_above_table(anchor, release_h_m, ctx), ctx)[0]
        ):
            return anchor, (dx, dy), scale
    return None


def _auto_solve_hover(
    cup_world_m: np.ndarray,
    *,
    requested_hover_h_m: float,
    grasp_h_m: float,
    release_h_m: float,
    place_offset_mm: tuple[float, float],
    ctx,
) -> tuple[float, np.ndarray, tuple[float, float], float]:
    min_h = grasp_h_m + MIN_HOVER_OVER_GRASP_MM * 1e-3
    h     = max(float(requested_hover_h_m), min_h)
    while h >= min_h - 1e-9:
        if _reach_ok(project_above_table(cup_world_m, h, ctx), ctx)[0]:
            place = _try_place_anchor(
                cup_world_m, place_offset_mm,
                hover_h_m=h, release_h_m=release_h_m, ctx=ctx,
            )
            if place is not None:
                return h, *place
        h -= 0.005
    raise ReachabilityError(
        f"No reachable hover+place combo from "
        f"{requested_hover_h_m*1000:.0f} mm down to {min_h*1000:.0f} mm. "
        f"Move cup closer to robot or reduce --place-offset-mm."
    )


# =============================================================================
# Main sequence
# =============================================================================
def pick_and_place(
    pose,
    *,
    profile_path: str  = DEFAULT_PROFILE,
    port: str          = "/dev/ttyUSB0",
    baudrate: int      = 1_000_000,
    speed: int         = 15,
    max_reach_mm: float = 350.0,
    hover_mm: float    = DEFAULT_HOVER_MM,
    safe_transit_mm: float = DEFAULT_SAFE_TRANSIT_MM,
    grasp_height_mm: float = DEFAULT_GRASP_HEIGHT_MM,
    release_mm: float  = DEFAULT_RELEASE_MM,
    place_offset_mm: tuple[float, float] = DEFAULT_PLACE_OFFSET_MM,
    grasp_offset_mm: tuple[float, float] = DEFAULT_GRASP_OFFSET_MM,
    grasp_close_value: int = DEFAULT_GRASP_CLOSE_VALUE,
    dry_run: bool      = False,
) -> None:
    cup_world_raw       = np.asarray(pose.position_world_m, dtype=np.float64)

    # ---- Grasp offset resolution ------------------------------------------
    # Two paths:
    #   A) CLI override: if grasp_offset_mm has any non-zero component,
    #      use it as-is. Manual control wins.
    #   B) Auto lookup: X = DEFAULT_GRASP_X_OFFSET_MM (always +4 by default).
    #      Y = looked up from GRASP_Y_LOOKUP_TABLE based on cup_world_raw[1].
    cli_offset = (float(grasp_offset_mm[0]), float(grasp_offset_mm[1]))
    if cli_offset[0] != 0.0 or cli_offset[1] != 0.0:
        # CLI manual override wins.
        effective_offset = cli_offset
        offset_source    = "CLI override (--grasp-offset-mm)"
    else:
        # Auto: both X and Y looked up from the per-bin table.
        cup_world_y_mm       = float(cup_world_raw[1] * 1000.0)
        x_off, y_off, dbg    = _lookup_grasp_offset_mm(cup_world_y_mm)
        effective_offset     = (x_off, y_off)
        offset_source        = (
            f"auto: cup_world_y={cup_world_y_mm:+.1f} mm, {dbg}"
        )

    print(
        f"[pick_top] grasp offset -> "
        f"({effective_offset[0]:+.1f}, {effective_offset[1]:+.1f}) mm  "
        f"[{offset_source}]"
    )

    # Apply to the cup XY for picking only.
    grasp_offset_world  = np.array(
        [effective_offset[0] * 1e-3, effective_offset[1] * 1e-3, 0.0],
        dtype=np.float64,
    )
    cup_world           = cup_world_raw + grasp_offset_world
    requested_hover_h_m = float(hover_mm)        * 1e-3
    safe_h_m            = float(safe_transit_mm) * 1e-3
    grasp_h_m           = float(grasp_height_mm) * 1e-3
    release_h_m         = float(release_mm)      * 1e-3

    # CRITICAL: the place anchor is computed from cup_world_raw (NOT the
    # shifted cup_world). The grasp offset is a correction for picking
    # only; the place location is the cup's true detected position plus
    # the user's place offset.

    # Safety: SAFE must be strictly above HOVER.
    if safe_h_m <= requested_hover_h_m:
        raise ValueError(
            f"--safe-transit-mm ({safe_transit_mm:.0f}) must be greater "
            f"than --hover-mm ({hover_mm:.0f}). Safe transit is meant to "
            f"be ABOVE the working hover."
        )

    ctx, t_robot_world = _build_context(
        profile_path, speed=speed, max_reach_mm=max_reach_mm
    )

    descend_pick_w = project_above_table(cup_world, grasp_h_m, ctx)
    _check_reach("DESCEND_PICK", descend_pick_w, ctx)

    # For auto-hover and place-anchor, use the RAW cup position so the
    # place location is relative to where the cup actually is, not where
    # the grasp offset wants us to descend.
    hover_h_m, place_anchor, place_offset_used, place_scale = _auto_solve_hover(
        cup_world_raw,
        requested_hover_h_m=requested_hover_h_m,
        grasp_h_m=grasp_h_m,
        release_h_m=release_h_m,
        place_offset_mm=place_offset_mm,
        ctx=ctx,
    )
    if hover_h_m < requested_hover_h_m - 1e-6:
        print(
            f"[pick_top] auto-hover: reduced to {hover_h_m*1000:.0f} mm "
            f"(requested {requested_hover_h_m*1000:.0f} mm unreachable)."
        )

    hover_pick_w    = project_above_table(cup_world,    hover_h_m,  ctx)
    hover_place_w   = project_above_table(place_anchor, hover_h_m,  ctx)
    descend_place_w = project_above_table(place_anchor, release_h_m, ctx)
    safe_pick_w     = project_above_table(cup_world,    safe_h_m,   ctx)
    safe_place_w    = project_above_table(place_anchor, safe_h_m,   ctx)

    waypoints = [
        ("0.5 SAFE_PICK   ", safe_pick_w),
        ("1   HOVER_PICK  ", hover_pick_w),
        ("2   DESCEND_PICK", descend_pick_w),
        ("4   LIFT_PICK   ", hover_pick_w),
        ("4.5 SAFE_PICK   ", safe_pick_w),
        ("5   SAFE_PLACE  ", safe_place_w),
        ("5b  HOVER_PLACE ", hover_place_w),
        ("6   DESCEND_PLA ", descend_place_w),
        ("8   LIFT_PLACE  ", hover_place_w),
        ("8b  SAFE_PLACE  ", safe_place_w),
    ]
    for label, p in waypoints:
        _check_reach(label, p, ctx)

    # Print plan.
    wx, wy, wz = cup_world * 1000.0
    cup_r_mm   = world_to_robot(cup_world, t_robot_world) * 1000.0
    print(
        f"\n[pick_top] cup world_mm  = ({wx:+.1f}, {wy:+.1f}, {wz:+.1f})\n"
        f"[pick_top] cup robot_mm  = ({cup_r_mm[0]:+.1f}, "
        f"{cup_r_mm[1]:+.1f}, {cup_r_mm[2]:+.1f})\n"
        f"[pick_top] RPY           = {TOP_DOWN_RPY}  (fixed, top-down)\n"
        f"[pick_top] altitudes     = SAFE {safe_h_m*1000:.0f} mm  /  "
        f"HOVER {hover_h_m*1000:.0f} mm  /  GRASP {grasp_height_mm:.0f} mm\n"
        f"[pick_top] grasp_offset  = ({effective_offset[0]:+.1f}, "
        f"{effective_offset[1]:+.1f}) mm world XY "
        f"({offset_source})\n"
        f"[pick_top] PLAN:\n"
        f"  0.   HOME            return to home joints\n"
        f"  0.5  SAFE_PICK       LONG horizontal at SAFE altitude "
        f"({safe_h_m*1000:.0f} mm above cup)\n"
        f"  1.   HOVER_PICK      vertical descent to "
        f"{hover_h_m*1000:.0f} mm above cup\n"
        f"  2.   DESCEND_PICK    vertical descent to "
        f"{grasp_height_mm:.0f} mm above table\n"
        f"  3.   CLOSE           gripper value={grasp_close_value}\n"
        f"  4.   LIFT_PICK       vertical lift to {hover_h_m*1000:.0f} mm\n"
        f"  4.5  SAFE_PICK       vertical lift to "
        f"{safe_h_m*1000:.0f} mm safe altitude\n"
        f"  5.   SAFE_PLACE      LONG horizontal at SAFE altitude to place XY\n"
        f"                       (place offset world "
        f"{place_offset_used[0]:+.0f}, {place_offset_used[1]:+.0f} mm, "
        f"{place_scale*100:.0f}%)\n"
        f"  5b.  HOVER_PLACE     vertical descent to "
        f"{hover_h_m*1000:.0f} mm above place\n"
        f"  6.   DESCEND_PLACE   vertical descent to "
        f"{release_mm:.0f} mm above table\n"
        f"  7.   OPEN            release\n"
        f"  8.   LIFT_PLACE      vertical lift to {hover_h_m*1000:.0f} mm\n"
        f"  8b.  SAFE_PLACE      vertical lift to "
        f"{safe_h_m*1000:.0f} mm safe altitude\n"
        f"  9.   HOME            return to home joints"
    )
    for label, p in waypoints:
        pr = world_to_robot(p, t_robot_world) * 1000.0
        print(
            f"  {label}: "
            f"world=({p[0]*1000:+.1f}, {p[1]*1000:+.1f}, {p[2]*1000:+.1f})"
            f"  robot=({pr[0]:+.1f}, {pr[1]:+.1f}, {pr[2]:+.1f}) mm"
        )

    if dry_run:
        print("[pick_top] --dry-run; not moving.")
        return

    # ---- LIVE EXECUTION ----
    driver = MyCobotDriver(
        MyCobotDriverSettings(port=port, baudrate=baudrate)
    )
    driver.connect()
    try:
        driver.power_on()
        time.sleep(0.5)
        gripper = Gripper(driver, GripperSettings())

        # Go home first, then open gripper so it's visibly open at home.
        _move_joints_and_wait(driver, "HOME", HOME_JOINTS, speed=30)
        print("[pick_top] opening gripper at home position...")
        gripper.open(wait=True)
        time.sleep(0.5)

        try:
            # --- APPROACH ---
            print(f"[pick_top] (0.5) SAFE_PICK transit -- "
                  f"{safe_h_m*1000:.0f} mm above cup "
                  f"(LONG horizontal at HIGH altitude)...")
            gripper.open(wait=False)
            move_to_world(driver, safe_pick_w, ctx, speed=int(speed))
            time.sleep(0.5)

            print(f"[pick_top] (1) HOVER_PICK -- "
                  f"vertical descent to {hover_h_m*1000:.0f} mm above cup...")
            move_to_world(driver, hover_pick_w, ctx, speed=int(speed))
            time.sleep(0.4)

            # --- PICK ---
            print(f"[pick_top] (2) DESCEND_PICK -- "
                  f"vertical descent to {grasp_height_mm:.0f} mm above table...")
            move_to_world(driver, descend_pick_w, ctx, speed=int(speed))
            time.sleep(0.5)

            print(f"[pick_top] (3) CLOSE gripper "
                  f"(value={grasp_close_value})...")
            gripper.close(value=int(grasp_close_value), wait=True)
            time.sleep(0.6)

            print(f"[pick_top] (4) LIFT_PICK -- "
                  f"vertical lift to {hover_h_m*1000:.0f} mm...")
            move_to_world(driver, hover_pick_w, ctx, speed=int(speed))
            time.sleep(0.4)

            print(f"[pick_top] (4.5) SAFE_PICK -- "
                  f"vertical lift to {safe_h_m*1000:.0f} mm safe altitude...")
            move_to_world(driver, safe_pick_w, ctx, speed=int(speed))
            time.sleep(0.4)

            # --- TRANSIT TO PLACE ---
            print(f"[pick_top] (5) SAFE_PLACE transit -- "
                  f"LONG horizontal at {safe_h_m*1000:.0f} mm safe altitude...")
            move_to_world(driver, safe_place_w, ctx, speed=int(speed))
            time.sleep(0.4)

            print(f"[pick_top] (5b) HOVER_PLACE -- "
                  f"vertical descent to {hover_h_m*1000:.0f} mm above place...")
            move_to_world(driver, hover_place_w, ctx, speed=int(speed))
            time.sleep(0.4)

            # --- PLACE ---
            print(f"[pick_top] (6) DESCEND_PLACE -- "
                  f"vertical descent to {release_mm:.0f} mm above table...")
            move_to_world(driver, descend_place_w, ctx, speed=int(speed))
            time.sleep(0.4)

            print("[pick_top] (7) OPEN -- release cup...")
            gripper.open(wait=True)
            time.sleep(0.4)

            print(f"[pick_top] (8) LIFT_PLACE -- "
                  f"vertical lift to {hover_h_m*1000:.0f} mm...")
            move_to_world(driver, hover_place_w, ctx, speed=int(speed))
            time.sleep(0.4)

            print(f"[pick_top] (8b) SAFE_PLACE -- "
                  f"vertical lift to {safe_h_m*1000:.0f} mm safe altitude...")
            move_to_world(driver, safe_place_w, ctx, speed=int(speed))
            time.sleep(0.4)

            print("[pick_top] done.")

        except Exception as exc:
            print(f"\n[pick_top] ERROR during execution: {exc}")
            print("[pick_top] resetting to home regardless...")

        finally:
            # Always reset -- success, crash, or Ctrl+C.
            print("[pick_top] RESET -- returning home and opening gripper...")
            try:
                _move_joints_and_wait(driver, "HOME", HOME_JOINTS, speed=30)
            except Exception:
                pass
            try:
                gripper.open(wait=True)
            except Exception:
                pass
            print("[pick_top] reset complete. Arm at HOME, gripper OPEN.")

    finally:
        driver.disconnect()


# =============================================================================
# CLI
# =============================================================================
def main() -> None:
    p = argparse.ArgumentParser(
        prog="python pick_place_cup_topdown.py",
        description=(
            "Top-down pick-and-place for a 35 mm cup. Two-tier altitude: "
            "SAFE transit for long horizontal moves, HOVER for the working "
            "altitude before grasp/release. Gripper always points straight "
            "down (RPY=180,0,0)."
        ),
    )
    p.add_argument("--profile",  default=DEFAULT_PROFILE)
    p.add_argument("--port",     default="/dev/ttyUSB0")
    p.add_argument("--baudrate", type=int, default=1_000_000)
    p.add_argument("--target-label",   default="cup")
    p.add_argument("--min-confidence", type=float,
                   default=DEFAULT_MIN_CONFIDENCE)
    p.add_argument("--settle-frames",  type=int,
                   default=DEFAULT_SETTLE_FRAMES)
    p.add_argument("--timeout-s",      type=float,
                   default=DEFAULT_TIMEOUT_S)
    p.add_argument("--device-index",   type=int, default=0)
    p.add_argument(
        "--hover-mm", type=float, default=DEFAULT_HOVER_MM,
        help=f"Working hover altitude above table "
             f"(default {DEFAULT_HOVER_MM:.0f} mm). The altitude RIGHT "
             f"BEFORE descending to grasp.",
    )
    p.add_argument(
        "--safe-transit-mm", type=float, default=DEFAULT_SAFE_TRANSIT_MM,
        help=f"SAFE transit altitude for long horizontal moves "
             f"(default {DEFAULT_SAFE_TRANSIT_MM:.0f} mm). MUST be greater "
             f"than --hover-mm. All long horizontal moves happen at this "
             f"HIGH altitude; only pure vertical descents go below it.",
    )
    p.add_argument(
        "--grasp-height-mm", type=float, default=DEFAULT_GRASP_HEIGHT_MM,
        help=f"Height above table to close gripper "
             f"(default {DEFAULT_GRASP_HEIGHT_MM:.0f} mm).",
    )
    p.add_argument(
        "--release-mm", type=float, default=DEFAULT_RELEASE_MM,
        help=f"Release height at place (default {DEFAULT_RELEASE_MM:.0f} mm).",
    )
    p.add_argument(
        "--place-offset-mm", type=float, nargs=2,
        default=list(DEFAULT_PLACE_OFFSET_MM),
        metavar=("DX", "DY"),
        help=f"Place location offset from cup XY in world mm "
             f"(default {DEFAULT_PLACE_OFFSET_MM[0]:+.0f} "
             f"{DEFAULT_PLACE_OFFSET_MM[1]:+.0f}).",
    )
    p.add_argument(
        "--grasp-offset-mm", type=float, nargs=2,
        default=list(DEFAULT_GRASP_OFFSET_MM),
        metavar=("DX", "DY"),
        help=f"Calibrated XY shift applied to the PICK target only "
             f"(default {DEFAULT_GRASP_OFFSET_MM[0]:+.1f} "
             f"{DEFAULT_GRASP_OFFSET_MM[1]:+.1f}). Use to correct "
             f"systematic grasp misalignment without re-calibrating. "
             f"Example: gripper hits +X side of cup rim "
             f"-> use --grasp-offset-mm -2 0 to shift the grasp target "
             f"2 mm in world -X. The PLACE location is unaffected.",
    )
    p.add_argument(
        "--grasp-close-value", type=int, default=DEFAULT_GRASP_CLOSE_VALUE,
        help=f"Gripper close value 0..100 "
             f"(default {DEFAULT_GRASP_CLOSE_VALUE}).",
    )
    p.add_argument("--speed",        type=int,   default=15)
    p.add_argument("--max-reach-mm", type=float, default=350.0)
    p.add_argument("--dry-run",      action="store_true")
    p.add_argument(
        "--cup-height-mm", type=float, default=DEFAULT_CUP_HEIGHT_MM,
        help=f"Known cup height in mm (default {DEFAULT_CUP_HEIGHT_MM:.0f}).",
    )
    args = p.parse_args()

    print(
        f"[pick_top] detecting {args.target_label!r} "
        f"(timeout {args.timeout_s:.1f}s)...",
        flush=True,
    )
    try:
        pose = locate_cup_robot_mm(
            args.profile,
            target_label=args.target_label,
            timeout_s=args.timeout_s,
            settle_frames=args.settle_frames,
            min_confidence=args.min_confidence,
            device_index=args.device_index,
        )
    except CupNotFoundError as exc:
        print(f"[pick_top] {exc}", file=sys.stderr)
        sys.exit(2)

    measured = (
        "n/a" if pose.height_m is None
        else f"{pose.height_m*1000:.1f} mm"
    )
    pose = with_height_override(pose, args.cup_height_mm)
    print(
        f"[pick_top] cup height: measured={measured} "
        f"-> using {args.cup_height_mm:.1f} mm"
    )

    try:
        pick_and_place(
            pose,
            profile_path=args.profile,
            port=args.port,
            baudrate=args.baudrate,
            speed=args.speed,
            max_reach_mm=args.max_reach_mm,
            hover_mm=args.hover_mm,
            safe_transit_mm=args.safe_transit_mm,
            grasp_height_mm=args.grasp_height_mm,
            release_mm=args.release_mm,
            place_offset_mm=(
                args.place_offset_mm[0], args.place_offset_mm[1]
            ),
            grasp_offset_mm=(
                args.grasp_offset_mm[0], args.grasp_offset_mm[1]
            ),
            grasp_close_value=args.grasp_close_value,
            dry_run=args.dry_run,
        )
    except (ValueError, ReachabilityError) as exc:
        print(f"[pick_top] {exc}", file=sys.stderr)
        sys.exit(3)


if __name__ == "__main__":
    main()