"""
Auto pick-and-place with GRIPPER FLIP at home AND settle verification.

Key fix vs previous version:
  - move_joints_and_verify() polls /joint_states until the arm has
    physically arrived at the target, instead of sleeping a fixed time.
  - This solves the problem where big joint moves (like the 180-degree
    J6 flip) didn't finish before the script moved on, leaving the arm
    in a weird in-between pose at the end.

Why the flip:
  The gripper has a servo housing on its underside. With the gripper in
  its natural orientation, that housing blocks reaching small/short
  objects close to the desk. Solution: rotate J6 by -180 degrees at
  home BEFORE approaching the cup.

Strategy:
  - Joint replay for the precise grab approach (taught with the flip)
  - Cartesian motion with orientation LOCKED for lift / swing / lower
    (keeps the gripper horizontal -- water wouldn't spill)
  - Place position = pick position mirrored across Y axis
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

# Cartesian motion settle is harder to verify (no precise position
# feedback in the same way), so we use a generous fixed sleep here.
CARTESIAN_WAIT_SEC = 5.0
CARTESIAN_MODE = 1

# How much to lift the cup vertically (mm).
LIFT_HEIGHT_MM = 80

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
    OR a timeout fires.

    This replaces the old time.sleep(WAIT_SEC) approach, which assumed
    every move took the same amount of time. Big moves (like a 180-deg
    rotation) need longer than small moves.
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
        # Compute the worst per-joint error.
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


def move_cartesian(mc, label, coords):
    print(f"\nMoving (Cartesian) to {label}: {coords}")
    mc.send_coords(coords, ARM_SPEED, CARTESIAN_MODE)
    time.sleep(CARTESIAN_WAIT_SEC)
    report_state(mc, f"After {label}")


def main():
    print("Connecting to robot...")
    port = find_robot_port()
    print("Using port:", port)
    mc = MyCobot(port, BAUD)
    time.sleep(1.0)

    report_state(mc, "Initial state")

    # Compute derived Cartesian waypoints.
    pick_lift_coords = list(PICK_GRAB_COORDS)
    pick_lift_coords[2] += LIFT_HEIGHT_MM

    place_lift_coords = list(pick_lift_coords)
    place_lift_coords[1] = -place_lift_coords[1]

    place_grab_coords = list(PICK_GRAB_COORDS)
    place_grab_coords[1] = -place_grab_coords[1]

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

    # 3. Flip gripper -180 degrees
    move_joints_and_verify(mc, "HOME_FLIPPED", HOME_FLIPPED_JOINTS)

    # 4. Approach the cup
    move_joints_and_verify(mc, "PICK_ABOVE", PICK_ABOVE_JOINTS)

    # 5. Move into the grab pose
    move_joints_and_verify(mc, "PICK_GRAB", PICK_GRAB_JOINTS)

    # 6. Grab
    set_gripper(mc, GRIPPER_CLOSED, "Closing")

    # 7. Lift vertically (Cartesian, orientation locked)
    move_cartesian(mc, "PICK_LIFT (vertical up)", pick_lift_coords)

    # 8. Swing across (Cartesian, orientation locked)
    move_cartesian(mc, "PLACE_LIFT (horizontal swing)", place_lift_coords)

    # 9. Lower (Cartesian, orientation locked)
    move_cartesian(mc, "PLACE_GRAB (vertical down)", place_grab_coords)

    # 10. Release
    set_gripper(mc, GRIPPER_OPEN, "Opening / releasing")

    # 11. Lift away (Cartesian, orientation locked)
    move_cartesian(mc, "PLACE_RELEASE (vertical up)", place_release_coords)

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