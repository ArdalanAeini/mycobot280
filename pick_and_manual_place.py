"""
Pick the cup automatically, then jog the arm with arrow keys to choose
where to place it. Press ENTER to lower, release, lift away, and go home.

Run this in a real SSH terminal (not a non-interactive runner):
  python3 pick_and_manual_place.py

Controls (myCobot frame: +X forward, +Y arm's left, +Z up):
  HOLD arrow keys   -> move continuously in X / Y (release to stop)
  HOLD u / j        -> move continuously in Z
  w/a/s/d           -> same as arrows (hold for continuous move)
  [ / ]             -> slower / faster jog speed
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

# Hold-to-move: velocity stream (no stop-start between steps).
JOG_VELOCITY_MM_S = 35           # how fast target moves while key held
JOG_VELOCITY_MIN = 10
JOG_VELOCITY_MAX = 70
JOG_STREAM_SPEED = 50            # send_coords speed (firmware units)
JOG_COMMAND_INTERVAL_SEC = 0.08  # command rate ~12 Hz (overlap motion)
JOG_SYNC_INTERVAL_SEC = 0.20     # read actual pose for display / stuck check
JOG_RELEASE_TIMEOUT_SEC = 0.30   # stop after no key event (SSH repeat varies)
JOG_STUCK_TIME_SEC = 0.7         # no motion this long -> out of reach
JOG_STUCK_MOVE_MM = 3.0          # min axis travel per sync to count as moving
KEY_POLL_MS = 30

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


def send_jog_command(mc, commanded):
    mc.send_coords(commanded, JOG_STREAM_SPEED, CARTESIAN_MODE)


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


def draw_screen(stdscr, actual, commanded, velocity, last_msg, held=None):
    stdscr.clear()
    h, w = stdscr.getmaxyx()
    lines = [
        "MANUAL PLACE MODE — cup is in the gripper",
        "",
        f"  X = {actual[0]:7.1f} mm   target {commanded[0]:7.1f}",
        f"  Y = {actual[1]:7.1f} mm   target {commanded[1]:7.1f}",
        f"  Z = {actual[2]:7.1f} mm   target {commanded[2]:7.1f}",
        f"  (+X forward, +Y arm left)  RX/RY/RZ locked",
        "",
        f"  Jog speed: {velocity:.0f} mm/s   ( [ slower , ] faster )",
        "",
        "  HOLD arrows / w/a/s/d  -> smooth X / Y",
        "  HOLD u / j             -> smooth Z",
        "  ENTER       -> place at target",
        "  q           -> quit (cup still held)",
        "",
        f"  Moving: {held[2] if held else '—'}",
        f"  {last_msg}",
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
    commanded = list(coords)
    actual = list(coords)
    velocity = float(JOG_VELOCITY_MM_S)
    last_msg = "HOLD arrow key to move"

    def _run(stdscr):
        nonlocal commanded, actual, velocity, last_msg
        curses.curs_set(0)
        stdscr.keypad(True)
        stdscr.timeout(KEY_POLL_MS)

        held = None
        last_input_time = 0.0
        last_loop_time = time.time()
        last_send_time = 0.0
        last_sync_time = 0.0
        stuck_time = 0.0
        prev_axis_pos = actual[0]

        while True:
            now = time.time()
            dt = now - last_loop_time
            last_loop_time = now
            if dt <= 0:
                dt = KEY_POLL_MS / 1000.0

            draw_screen(stdscr, actual, commanded, velocity, last_msg, held)

            while True:
                key = stdscr.getch()
                if key == -1:
                    break

                if key in MOVE_KEY_MAP:
                    if held is None:
                        snap = read_coords(mc)
                        if snap is not None:
                            actual = snap
                            commanded = list(snap)
                            lock_orientation(commanded, ref_orientation)
                            lock_orientation(actual, ref_orientation)
                    held = MOVE_KEY_MAP[key]
                    last_input_time = now
                    stuck_time = 0.0
                    prev_axis_pos = actual[held[0]]
                    last_msg = f"moving {held[2]}..."
                elif key == ord("["):
                    velocity = max(JOG_VELOCITY_MIN, velocity - 5)
                    last_msg = f"speed {velocity:.0f} mm/s"
                elif key == ord("]"):
                    velocity = min(JOG_VELOCITY_MAX, velocity + 5)
                    last_msg = f"speed {velocity:.0f} mm/s"
                elif key in (10, 13, curses.KEY_ENTER):
                    lock_orientation(commanded, ref_orientation)
                    return list(commanded)
                elif key in (ord("q"), ord("Q"), 27):
                    return None

            if held is not None and (now - last_input_time) > JOG_RELEASE_TIMEOUT_SEC:
                held = None
                snap = read_coords(mc)
                if snap is not None:
                    actual = snap
                    commanded = list(snap)
                    lock_orientation(commanded, ref_orientation)
                    lock_orientation(actual, ref_orientation)
                last_msg = "stopped"

            if held is not None:
                axis, sign, label = held
                commanded[axis] += sign * velocity * dt
                lock_orientation(commanded, ref_orientation)

                if now - last_send_time >= JOG_COMMAND_INTERVAL_SEC:
                    send_jog_command(mc, commanded)
                    last_send_time = now

            if now - last_sync_time >= JOG_SYNC_INTERVAL_SEC:
                snap = read_coords(mc)
                if snap is not None:
                    actual = snap
                    lock_orientation(actual, ref_orientation)
                    if held is not None:
                        axis, sign, label = held
                        moved = abs(actual[axis] - prev_axis_pos)
                        if moved < JOG_STUCK_MOVE_MM:
                            stuck_time += JOG_SYNC_INTERVAL_SEC
                        else:
                            stuck_time = 0.0
                        prev_axis_pos = actual[axis]
                        if stuck_time >= JOG_STUCK_TIME_SEC:
                            held = None
                            commanded = list(actual)
                            last_msg = f"BLOCKED {label} — out of reach"
                last_sync_time = now

    print("\n" + "=" * 50)
    print("Manual jog — HOLD arrow keys (smooth velocity mode).")
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
