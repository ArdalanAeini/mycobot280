"""
Auto pick-and-place with GRIPPER FLIP at home, settle verification,
and configurable place offset.

Strategy:
  - Joint replay for the precise grab approach (taught with the flip)
  - Cartesian motion with orientation LOCKED for lift / swing / lower
    (keeps the gripper horizontal -- water in the cup wouldn't spill)
  - Place position = pick position + configurable (X, Y) offset

Why the flip:
  The gripper has a servo housing on its underside. With the gripper in
  its natural orientation, that housing blocks reaching small/short
  objects close to the desk. Solution: rotate J6 by +180 degrees at
  home BEFORE approaching the cup.

Why settle verification:
  send_angles() returns immediately; the motors physically need time.
  A fixed sleep is unreliable for big joint moves (180-degree rotations
  take longer than small ones). We poll get_angles() until the arm has
  ARRIVED within JOINT_TOLERANCE_DEG of the target.
"""

import time
import glob
from pymycobot.mycobot import MyCobot


BAUD = 1000000

ARM_SPEED = 25
GRIPPER_SPEED = 50
GRIPPER_OPEN = 100
GRIPPER_CLOSED = 0

GRIPPER_WAIT_SEC = 1.5

# Settle parameters: when sending a joint command, poll the actual
# joint state until it's within JOINT_TOLERANCE_DEG of target, or
# until SETTLE_TIMEOUT_SEC has elapsed.
JOINT_TOLERANCE_DEG = 3.0
SETTLE_TIMEOUT_SEC = 12.0
POLL_INTERVAL_SEC = 0.2

# Cartesian: send_coords() returns immediately even if IK failed (silent).
# Poll get_coords() until XYZ is within tolerance or timeout.
COORD_TOLERANCE_MM = 15.0
CARTESIAN_SETTLE_TIMEOUT_SEC = 15.0
CARTESIAN_MODE = 1   # 1 = linear (straight-line in 3D space)

# How much to lift the cup vertically (mm).
LIFT_HEIGHT_MM = 80

# Place offset: where the cup ends up, relative to the pick position.
# Adjust these to choose where the cup gets placed.
#   +X = forward (away from operator), -X = backward (toward operator)
#   +Y = arm's left, -Y = arm's right
# Example values:
#   (0, 150)    -> 150 mm left  (known good on this setup)
#   (0, -150)   -> 150 mm right — often OUT OF REACH at pick orientation; arm won't move
#   (-150, 0)   -> 150 mm toward operator (other side of table along X)
#   (150, 0)    -> 150 mm away from operator
PLACE_OFFSET_X_MM = -150
PLACE_OFFSET_Y_MM = 0

# Home poses.
HOME_JOINTS         = [0, 0, 0, 0, 0,    0]
HOME_FLIPPED_JOINTS = [0, 0, 0, 0, 0, +180]

# Taught positions from teach_pickup_pair.py.
PICK_ABOVE_JOINTS = [-21.09, -97.20, -0.79, -88.41, 105.90, -10.10]
PICK_GRAB_JOINTS  = [-15.02, -97.29, -0.79, -84.46, 109.59,  -5.00]

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
    """
    Send a joint command and POLL until the arm has physically arrived,
    OR a timeout fires. Solves the problem where fixed sleeps weren't
    enough for big joint moves like the 180-degree gripper flip.
    """
    print(f"\nMoving (joint) to {label}: {target}")
    mc.send_angles(target, ARM_SPEED)

    deadline = time.time() + SETTLE_TIMEOUT_SEC
    last_err = float("inf")
    while time.time() < deadline:
        time.sleep(POLL_INTERVAL_SEC)
        actual = mc.get_angles()
        if actual is None or len(actual) == 0:
            continue
        per_joint_err = [abs(a - t) for a, t in zip(actual, target)]
        last_err = max(per_joint_err)
        if last_err < JOINT_TOLERANCE_DEG:
            elapsed = SETTLE_TIMEOUT_SEC - (deadline - time.time())
            print(f"  SETTLED at {label} after {elapsed:.1f}s, "
                  f"max joint err {last_err:.2f} deg")
            report_state(mc, f"After {label}")
            return True

    print(f"  TIMED OUT moving to {label} after {SETTLE_TIMEOUT_SEC}s, "
          f"final max joint err {last_err:.2f} deg")
    report_state(mc, f"After {label} (timed out)")
    return False


def move_cartesian_and_verify(mc, label, target):
    """
    Send Cartesian target and poll until XYZ matches within tolerance.
    pymycobot does not raise if IK fails — without this, the script would
    keep going and release the cup without ever swinging to the place pose.
    """
    print(f"\nMoving (Cartesian) to {label}: {target}")
    mc.send_coords(target, ARM_SPEED, CARTESIAN_MODE)

    deadline = time.time() + CARTESIAN_SETTLE_TIMEOUT_SEC
    last_err = float("inf")
    while time.time() < deadline:
        time.sleep(POLL_INTERVAL_SEC)
        actual = mc.get_coords()
        if actual is None or len(actual) < 3:
            continue
        pos_err = max(abs(a - t) for a, t in zip(actual[:3], target[:3]))
        last_err = pos_err
        if pos_err < COORD_TOLERANCE_MM:
            elapsed = CARTESIAN_SETTLE_TIMEOUT_SEC - (deadline - time.time())
            print(f"  SETTLED at {label} after {elapsed:.1f}s, max XYZ err {last_err:.1f} mm")
            report_state(mc, f"After {label}")
            return True

    print(f"  FAILED at {label} after {CARTESIAN_SETTLE_TIMEOUT_SEC}s — "
          f"max XYZ err {last_err:.1f} mm (target likely unreachable)")
    report_state(mc, f"After {label} (FAILED)")
    return False


def main():
    print("Connecting to robot...")
    port = find_robot_port()
    print("Using port:", port)
    mc = MyCobot(port, BAUD)
    time.sleep(1.0)

    report_state(mc, "Initial state")

    # Compute derived Cartesian waypoints.
    # Pick lift: same XY as PICK_GRAB, Z raised so the cup clears the table.
    pick_lift_coords = list(PICK_GRAB_COORDS)
    pick_lift_coords[2] += LIFT_HEIGHT_MM

    # Place grab: PICK_GRAB shifted by configurable X and Y offsets.
    place_grab_coords = list(PICK_GRAB_COORDS)
    place_grab_coords[0] += PLACE_OFFSET_X_MM
    place_grab_coords[1] += PLACE_OFFSET_Y_MM

    # Place lift: place_grab with Z raised, so we can swing safely.
    place_lift_coords = list(place_grab_coords)
    place_lift_coords[2] += LIFT_HEIGHT_MM

    # Place release-and-back-away: same as place_grab but Z lifted.
    place_release_coords = list(place_grab_coords)
    place_release_coords[2] += LIFT_HEIGHT_MM

    print("\nComputed Cartesian waypoints:")
    print(f"  PICK_LIFT:     {[round(c, 1) for c in pick_lift_coords]}")
    print(f"  PLACE_LIFT:    {[round(c, 1) for c in place_lift_coords]}")
    print(f"  PLACE_GRAB:    {[round(c, 1) for c in place_grab_coords]}")
    print(f"  PLACE_RELEASE: {[round(c, 1) for c in place_release_coords]}")

    print("\nKeep your hand near power/E-stop.")
    input("Press ENTER to start...")

    # ----- THE SEQUENCE -----

    # 1. Open gripper
    set_gripper(mc, GRIPPER_OPEN, "Opening")

    # 2. Go HOME (natural orientation)
    move_joints_and_verify(mc, "HOME", HOME_JOINTS)

    # 3. Flip gripper +180 degrees so the slim side faces down
    move_joints_and_verify(mc, "HOME_FLIPPED", HOME_FLIPPED_JOINTS)

    # 4. Approach the cup
    move_joints_and_verify(mc, "PICK_ABOVE", PICK_ABOVE_JOINTS)

    # 5. Move into the grab pose
    move_joints_and_verify(mc, "PICK_GRAB", PICK_GRAB_JOINTS)

    # 6. Grab
    set_gripper(mc, GRIPPER_CLOSED, "Closing")

    # 7. Lift vertically (Cartesian, orientation locked)
    if not move_cartesian_and_verify(mc, "PICK_LIFT (vertical up)", pick_lift_coords):
        print("\nABORT: pick lift failed. Cup still in gripper — adjust pose or offsets.")
        return

    # 8. Swing horizontally to (X+offset_x, Y+offset_y) (Cartesian, orientation locked)
    if not move_cartesian_and_verify(mc, "PLACE_LIFT (horizontal swing)", place_lift_coords):
        print("\nABORT: place swing failed (often out of reach). Cup still in gripper.")
        print(f"  Tried place at X+{PLACE_OFFSET_X_MM}, Y+{PLACE_OFFSET_Y_MM} mm from pick.")
        print("  Try smaller offsets, or (0, 150) for left side, or (-120, 0) toward you.")
        return

    # 9. Lower (Cartesian, orientation locked)
    if not move_cartesian_and_verify(mc, "PLACE_GRAB (vertical down)", place_grab_coords):
        print("\nABORT: place lower failed. Cup still in gripper.")
        return

    # 10. Release
    set_gripper(mc, GRIPPER_OPEN, "Opening / releasing")

    # 11. Lift away (Cartesian, orientation locked)
    if not move_cartesian_and_verify(mc, "PLACE_RELEASE (vertical up)", place_release_coords):
        print("\nWARNING: release lift failed; continuing to joint homing.")

    # 12. Un-flip before going home
    move_joints_and_verify(mc, "HOME_FLIPPED (return through flipped pose)",
                           HOME_FLIPPED_JOINTS)

    # 13. Return HOME
    move_joints_and_verify(mc, "HOME", HOME_JOINTS)

    print("\n" + "=" * 50)
    print("Pick-and-place complete.")
    print("=" * 50)


if __name__ == "__main__":
    main()