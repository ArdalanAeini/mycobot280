"""
Pick the cup automatically, then jog the arm with arrow keys to choose
where to place it. Press ENTER to lower, release, lift away, and go home.

Run this in a real SSH terminal (not a non-interactive runner):
  python3 pick_and_manual_place.py

Controls (myCobot frame: +X forward, +Y arm's left, +Z up):
  HOLD arrow keys   -> move continuously in X / Y (release to stop)
  HOLD u / j        -> move continuously in Z
  w/a/s/d           -> same as arrows (hold for continuous move)
  [ / ]             -> smaller / larger step per nudge
  ENTER             -> place cup here (lower, open gripper, lift, home)
  q                 -> quit jog mode (cup still held; you handle recovery)

Why Cartesian jog (not joint jog):
  Orientation stays locked (RX/RY/RZ fixed) so the cup stays level.
"""

import sys
import time
import glob
import curses

from pymycobot.mycobot import MyCobot


BAUD = 1000000
ARM_SPEED = 25
JOG_SPEED = 35
GRIPPER_SPEED = 50
GRIPPER_OPEN = 100
GRIPPER_CLOSED = 0
GRIPPER_WAIT_SEC = 1.5

JOINT_TOLERANCE_DEG = 3.0
SETTLE_TIMEOUT_SEC = 12.0
POLL_INTERVAL_SEC = 0.2

COORD_TOLERANCE_MM = 15.0
CARTESIAN_SETTLE_TIMEOUT_SEC = 15.0
CARTESIAN_MODE = 1

LIFT_HEIGHT_MM = 80
JOG_STEP_MM = 10
JOG_STEP_MIN_MM = 2
JOG_STEP_MAX_MM = 40

# Hold-to-move timing (tune if motion feels too fast/slow).
JOG_REPEAT_INTERVAL_SEC = 0.35   # send next nudge while key held
JOG_RELEASE_TIMEOUT_SEC = 0.18   # stop if no move key seen for this long
JOG_NUDGE_WAIT_SEC = 0.45        # wait after each nudge before reading pose
JOG_BLOCKED_MM = 22.0            # if XYZ err larger, IK likely failed
KEY_POLL_MS = 50                 # terminal poll interval

HOME_JOINTS = [0, 0, 0, 0, 0, 0]
HOME_FLIPPED_JOINTS = [0, 0, 0, 0, 0, +180]

PICK_ABOVE_JOINTS = [-21.09, -97.20, -0.79, -88.41, 105.90, -10.10]
PICK_GRAB_JOINTS = [-15.02, -97.29, -0.79, -84.46, 109.59, -5.00]
PICK_GRAB_COORDS = [205.2, -74.5, 29.8, 90.85, 2.60, 145.42]


def find_robot_port():
    ports = sorted(glob.glob("/dev/ttyUSB*"))
    if not ports:
        raise RuntimeError("No /dev/ttyUSB* device. Check arm power and USB cable.")
    print("Available USB ports:", ports)
    return ports[0]


def report_state(mc, label):
    print("\n" + "=" * 50)
    print(label)
    print("=" * 50)
    angles = mc.get_angles()
    coords = mc.get_coords()
    if angles:
        print("Joints:", [round(a, 2) for a in angles])
    if coords:
        print("Coords:", [round(c, 2) for c in coords])


def set_gripper(mc, value, label):
    print(f"\n{label} gripper: value={value}")
    mc.set_gripper_value(value, GRIPPER_SPEED)
    time.sleep(GRIPPER_WAIT_SEC)


def move_joints_and_verify(mc, label, target):
    print(f"\nMoving (joint) to {label}: {target}")
    mc.send_angles(target, ARM_SPEED)

    deadline = time.time() + SETTLE_TIMEOUT_SEC
    last_err = float("inf")
    while time.time() < deadline:
        time.sleep(POLL_INTERVAL_SEC)
        actual = mc.get_angles()
        if actual is None or len(actual) == 0:
            continue
        last_err = max(abs(a - t) for a, t in zip(actual, target))
        if last_err < JOINT_TOLERANCE_DEG:
            print(f"  SETTLED at {label}, max joint err {last_err:.2f} deg")
            return True

    print(f"  TIMED OUT at {label}, max joint err {last_err:.2f} deg")
    return False


def move_cartesian_and_verify(mc, label, target, speed=ARM_SPEED,
                              timeout=CARTESIAN_SETTLE_TIMEOUT_SEC,
                              tolerance=COORD_TOLERANCE_MM):
    print(f"\nMoving (Cartesian) to {label}")
    mc.send_coords(target, speed, CARTESIAN_MODE)

    deadline = time.time() + timeout
    last_err = float("inf")
    while time.time() < deadline:
        time.sleep(POLL_INTERVAL_SEC)
        actual = mc.get_coords()
        if actual is None or len(actual) < 3:
            continue
        last_err = max(abs(a - t) for a, t in zip(actual[:3], target[:3]))
        if last_err < tolerance:
            print(f"  SETTLED at {label}, max XYZ err {last_err:.1f} mm")
            return True

    print(f"  FAILED at {label}, max XYZ err {last_err:.1f} mm")
    return False


def read_coords(mc):
    coords = mc.get_coords()
    if coords is None or len(coords) < 6:
        return None
    return [float(c) for c in coords]


def lock_orientation(target, ref):
    """Keep cup level: copy RX/RY/RZ from reference pose."""
    target[3] = ref[3]
    target[4] = ref[4]
    target[5] = ref[5]
    return target


def jog_nudge(mc, coords, axis, sign, step_mm, ref_orientation):
    """
    One small Cartesian step for hold-to-move. Uses a short wait instead of
    long settle polling so repeated nudges feel continuous while a key is held.
    """
    target = list(coords)
    target[axis] += sign * step_mm
    lock_orientation(target, ref_orientation)

    mc.send_coords(target, JOG_SPEED, CARTESIAN_MODE)
    time.sleep(JOG_NUDGE_WAIT_SEC)

    actual = read_coords(mc)
    if actual is None:
        return False, coords, "no coords"

    lock_orientation(actual, ref_orientation)
    err = max(abs(a - t) for a, t in zip(actual[:3], target[:3]))
    if err > JOG_BLOCKED_MM:
        return False, actual, "blocked"

    return True, actual, "ok"


# (axis, sign, label) for held keys — axis 0=X, 1=Y, 2=Z
MOVE_KEY_MAP = {
    curses.KEY_UP: (0, +1, "+X"),
    curses.KEY_DOWN: (0, -1, "-X"),
    curses.KEY_RIGHT: (1, +1, "+Y"),
    curses.KEY_LEFT: (1, -1, "-Y"),
    ord("w"): (0, +1, "+X"),
    ord("s"): (0, -1, "-X"),
    ord("d"): (1, +1, "+Y"),
    ord("a"): (1, -1, "-Y"),
    ord("u"): (2, +1, "+Z"),
    ord("j"): (2, -1, "-Z"),
}


def draw_screen(stdscr, coords, step_mm, last_msg, held=None):
    stdscr.clear()
    h, w = stdscr.getmaxyx()
    lines = [
        "MANUAL PLACE MODE — cup is in the gripper",
        "",
        f"  X = {coords[0]:7.1f} mm   (+X = forward, away from you)",
        f"  Y = {coords[1]:7.1f} mm   (+Y = arm's left)",
        f"  Z = {coords[2]:7.1f} mm",
        f"  RX/RY/RZ locked for level carry",
        "",
        f"  Step size: {step_mm} mm per nudge  ( [ smaller , ] larger )",
        "",
        "  HOLD arrows / w/a/s/d  -> move X / Y continuously",
        "  HOLD u / j             -> Z up / down continuously",
        "  ENTER       -> place cup here",
        "  q           -> quit (cup still held)",
        "",
        f"  Moving: {held[2] if held else '—'}",
        f"  Last: {last_msg}",
    ]
    for i, line in enumerate(lines):
        if i >= h - 1:
            break
        stdscr.addstr(i, 0, line[: max(0, w - 1)])
    stdscr.refresh()


def manual_jog_loop(mc, ref_orientation):
    coords = read_coords(mc)
    if coords is None:
        print("Could not read coords after pick lift.")
        return None

    lock_orientation(coords, ref_orientation)
    step_mm = JOG_STEP_MM
    last_msg = "ready"

    def _run(stdscr):
        nonlocal coords, step_mm, last_msg
        curses.curs_set(0)
        stdscr.keypad(True)
        stdscr.timeout(KEY_POLL_MS)

        held = None
        last_input_time = 0.0
        last_jog_time = 0.0

        while True:
            now = time.time()
            draw_screen(stdscr, coords, step_mm, last_msg, held)

            # Read every pending key (terminal may repeat while held).
            while True:
                key = stdscr.getch()
                if key == -1:
                    break

                if key in MOVE_KEY_MAP:
                    held = MOVE_KEY_MAP[key]
                    last_input_time = now
                elif key == ord("["):
                    step_mm = max(JOG_STEP_MIN_MM, step_mm - 2)
                    last_msg = f"step -> {step_mm} mm"
                elif key == ord("]"):
                    step_mm = min(JOG_STEP_MAX_MM, step_mm + 2)
                    last_msg = f"step -> {step_mm} mm"
                elif key in (10, 13, curses.KEY_ENTER):
                    lock_orientation(coords, ref_orientation)
                    return coords
                elif key in (ord("q"), ord("Q"), 27):
                    return None

            if held is not None and (now - last_input_time) > JOG_RELEASE_TIMEOUT_SEC:
                held = None
                last_msg = "stopped (key released)"

            if held is not None and (now - last_jog_time) >= JOG_REPEAT_INTERVAL_SEC:
                axis, sign, label = held
                ok, actual, status = jog_nudge(
                    mc, coords, axis, sign, step_mm, ref_orientation
                )
                last_jog_time = now
                if actual is not None:
                    coords = actual
                if ok:
                    last_msg = f"moving {label} {step_mm}mm"
                else:
                    held = None
                    if status == "blocked":
                        last_msg = f"BLOCKED {label} — out of reach"
                    else:
                        last_msg = f"BLOCKED {label}"

    print("\n" + "=" * 50)
    print("Manual jog — HOLD arrow keys for continuous motion.")
    print("Click this terminal window so key presses go to SSH.")
    print("=" * 50)
    return curses.wrapper(_run)


def run_automated_pick(mc):
    pick_lift = list(PICK_GRAB_COORDS)
    pick_lift[2] += LIFT_HEIGHT_MM

    set_gripper(mc, GRIPPER_OPEN, "Opening")
    if not move_joints_and_verify(mc, "HOME", HOME_JOINTS):
        return False
    if not move_joints_and_verify(mc, "HOME_FLIPPED", HOME_FLIPPED_JOINTS):
        return False
    if not move_joints_and_verify(mc, "PICK_ABOVE", PICK_ABOVE_JOINTS):
        return False
    if not move_joints_and_verify(mc, "PICK_GRAB", PICK_GRAB_JOINTS):
        return False

    set_gripper(mc, GRIPPER_CLOSED, "Closing")

    if not move_cartesian_and_verify(mc, "PICK_LIFT", pick_lift):
        print("ABORT: pick lift failed.")
        return False

    return True


def run_place_at_current(mc, carry_coords):
    """Lower at current XY, release, lift, un-flip, home."""
    ref = PICK_GRAB_COORDS

    place_down = list(carry_coords)
    lock_orientation(place_down, ref)
    place_down[2] = PICK_GRAB_COORDS[2]

    place_up = list(place_down)
    place_up[2] += LIFT_HEIGHT_MM

    if not move_cartesian_and_verify(mc, "PLACE_DOWN", place_down):
        print("ABORT: could not lower to place height. Cup still held.")
        return False

    set_gripper(mc, GRIPPER_OPEN, "Releasing")

    if not move_cartesian_and_verify(mc, "PLACE_LIFT_AWAY", place_up):
        print("WARNING: lift after release failed; continuing to homing.")

    move_joints_and_verify(mc, "HOME_FLIPPED", HOME_FLIPPED_JOINTS)
    move_joints_and_verify(mc, "HOME", HOME_JOINTS)
    return True


def main():
    if not sys.stdin.isatty():
        print("ERROR: Run in an interactive SSH terminal (stdin must be a TTY).")
        print("  ssh jetson")
        print("  cd ~/src")
        print("  python3 pick_and_manual_place.py")
        sys.exit(1)

    print("Connecting to robot...")
    port = find_robot_port()
    mc = MyCobot(port, BAUD)
    time.sleep(1.0)
    report_state(mc, "Initial state")

    print("\nKeep hand near E-stop.")
    input("Press ENTER to start automated pick...")

    if not run_automated_pick(mc):
        return

    print("\nPick complete. Starting manual jog...")
    carry_pose = manual_jog_loop(mc, PICK_GRAB_COORDS)

    if carry_pose is None:
        print("\nQuit without placing. Cup may still be in gripper.")
        print("Move arm carefully or re-run script.")
        return

    print("\nPlacing at:")
    print("  ", [round(c, 1) for c in carry_pose])
    input("Press ENTER to lower and release here...")

    if run_place_at_current(mc, carry_pose):
        print("\nPick + manual place complete.")
    else:
        print("\nPlace sequence did not finish.")


if __name__ == "__main__":
    main()
