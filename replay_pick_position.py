"""
Joint-based hardcoded pick-and-place test.

This script uses taught joint positions only.

Sequence:
  1. Open gripper
  2. Go HOME
  3. Rotate gripper
  4. Move to pickup approach
  5. Move to pickup position
  6. Close gripper
  7. Lift
  8. Move to place approach
  9. Lower to place position
  10. Open gripper
  11. Lift away
  12. Return HOME
"""

import time
import glob
from pymycobot.mycobot import MyCobot


BAUD = 1000000

ARM_SPEED = 12
GRIPPER_SPEED = 50

GRIPPER_OPEN = 100
GRIPPER_CLOSED = 0

WAIT_SEC = 5.0
GRIPPER_WAIT_SEC = 1.5

# Safe home / park pose
HOME_JOINTS = [0, 0, 0, 0, 90, 0]

# Your taught pickup position
PICK_JOINTS = [-17.84, -125.24, -0.79, 43.15, 90.08, 98.7]

# Rotate gripper while still away from the cup
ROTATED_HOME_JOINTS = [0, 0, 0, 0, 90, 98.7]

# Approach above/near pickup.
# This is safer than going directly to PICK_JOINTS.
PICK_APPROACH_JOINTS = [-17.84, -95.0, -0.79, 43.15, 90.08, 98.7]

# Lift after grabbing.
PICK_LIFT_JOINTS = PICK_APPROACH_JOINTS.copy()

# Place position.
# For first test, we only change J1/base angle.
# This moves the cup sideways to another reachable location.
PLACE_JOINTS = [10.0, -125.24, -0.79, 43.15, 90.08, 98.7]

# Approach above the place position.
PLACE_APPROACH_JOINTS = [10.0, -95.0, -0.79, 43.15, 90.08, 98.7]

# Lift after releasing.
PLACE_LIFT_JOINTS = PLACE_APPROACH_JOINTS.copy()


def find_robot_port():
    ports = sorted(glob.glob("/dev/ttyUSB*"))
    if not ports:
        raise RuntimeError("No /dev/ttyUSB* device found. Check robot power and USB cable.")
    print("Available USB ports:", ports)
    return ports[0]


def report_state(mc, label):
    print("\n" + "=" * 60)
    print(label)
    print("=" * 60)

    angles = mc.get_angles()
    coords = mc.get_coords()

    if angles is None or len(angles) == 0:
        print("Joints: <could not read>")
    else:
        print("Joints:", [round(a, 2) for a in angles])

    if coords is None or len(coords) == 0:
        print("Coords: <could not read>")
    else:
        print("Coords:", [round(c, 2) for c in coords])


def set_gripper(mc, value, label):
    print(f"\n--- {label} gripper: value={value} ---")
    mc.set_gripper_value(value, GRIPPER_SPEED)
    time.sleep(GRIPPER_WAIT_SEC)

    try:
        val = mc.get_gripper_value()
        print("Gripper value:", val)
    except Exception:
        print("Could not read gripper value.")


def move_joints(mc, label, joints):
    print(f"\n--- Moving to {label} ---")
    print("Target joints:", joints)

    mc.send_angles(joints, ARM_SPEED)
    time.sleep(WAIT_SEC)

    report_state(mc, f"After {label}")


def main():
    print("Connecting to robot...")
    port = find_robot_port()
    print("Using port:", port)

    mc = MyCobot(port, BAUD)
    time.sleep(1.0)

    report_state(mc, "Initial state")

    print("\nThis script will pick the cup and place it nearby.")
    print("Keep your hand near power/emergency stop.")
    input("Press ENTER to start...")

    # 1. Open gripper
    set_gripper(mc, GRIPPER_OPEN, "Opening")

    # 2. Go home
    move_joints(mc, "HOME_JOINTS", HOME_JOINTS)

    # 3. Rotate gripper while away from object
    move_joints(mc, "ROTATED_HOME_JOINTS", ROTATED_HOME_JOINTS)

    # 4. Move near pickup
    move_joints(mc, "PICK_APPROACH_JOINTS", PICK_APPROACH_JOINTS)

    answer = input("\nIs the gripper safely near/above the cup? Type yes to continue: ").strip().lower()
    if answer != "yes":
        print("Stopped before pickup.")
        return

    # 5. Move to exact pickup position
    move_joints(mc, "PICK_JOINTS", PICK_JOINTS)

    answer = input("\nIs the gripper around the cup body? Type yes to close: ").strip().lower()
    if answer != "yes":
        print("Stopped before closing.")
        return

    # 6. Close gripper
    set_gripper(mc, GRIPPER_CLOSED, "Closing")

    answer = input("\nLift the cup? Type yes to lift: ").strip().lower()
    if answer != "yes":
        print("Stopped after gripping.")
        return

    # 7. Lift cup
    move_joints(mc, "PICK_LIFT_JOINTS", PICK_LIFT_JOINTS)

    # 8. Move to place approach
    move_joints(mc, "PLACE_APPROACH_JOINTS", PLACE_APPROACH_JOINTS)

    answer = input("\nIs the place location safe? Type yes to lower cup: ").strip().lower()
    if answer != "yes":
        print("Stopped before placing.")
        return

    # 9. Lower to place position
    move_joints(mc, "PLACE_JOINTS", PLACE_JOINTS)

    answer = input("\nOpen gripper to release cup? Type yes: ").strip().lower()
    if answer != "yes":
        print("Stopped before release.")
        return

    # 10. Release cup
    set_gripper(mc, GRIPPER_OPEN, "Opening / releasing")

    # 11. Lift away
    move_joints(mc, "PLACE_LIFT_JOINTS", PLACE_LIFT_JOINTS)

    # 12. Return home
    move_joints(mc, "HOME_JOINTS", HOME_JOINTS)

    print("\nPick-and-place test complete.")


if __name__ == "__main__":
    main()