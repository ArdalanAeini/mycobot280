"""
Joint-based hardcoded cup pickup test.

This version does NOT use send_coords().
It uses the manually taught PICK_JOINTS directly.

Sequence:
  1. Open gripper
  2. Go to HOME
  3. Rotate/prepare gripper away from cup
  4. Move to an approach pose near/above the cup
  5. Move to exact taught pickup joint pose
  6. Close gripper
  7. Lift using a safer joint pose

This is more reliable than Cartesian mode for now.
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

# Your taught pickup pose
PICK_JOINTS = [-17.84, -125.24, -0.79, 43.15, 90.08, 98.7]

# Same gripper rotation as pickup, but still at home.
ROTATED_HOME_JOINTS = [0, 0, 0, 0, 90, 98.7]

# Approach pose using joints.
# This keeps J1/J5/J6/J4 similar to pickup, but makes J2 less deep.
# If this still approaches too much from the side, we will tune this one pose.
APPROACH_JOINTS = [-17.84, -95.0, -0.79, 43.15, 90.08, 98.7]

# Lift pose after grabbing.
# Same as approach for first test.
LIFT_JOINTS = APPROACH_JOINTS.copy()


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

    print("\nThis script uses JOINTS ONLY.")
    print("Keep your hand near power/emergency stop.")
    input("Press ENTER to start...")

    # 1. Open gripper
    set_gripper(mc, GRIPPER_OPEN, "Opening")

    # 2. Go home
    move_joints(mc, "HOME_JOINTS", HOME_JOINTS)

    # 3. Rotate gripper while away from cup
    move_joints(mc, "ROTATED_HOME_JOINTS", ROTATED_HOME_JOINTS)

    # 4. Move to approach pose
    move_joints(mc, "APPROACH_JOINTS", APPROACH_JOINTS)

    answer = input("\nIs the gripper safely near/above the cup? Type yes to go to pickup: ").strip().lower()
    if answer != "yes":
        print("Stopped before pickup.")
        print("We need to tune APPROACH_JOINTS.")
        return

    # 5. Move to exact taught pickup pose
    move_joints(mc, "PICK_JOINTS", PICK_JOINTS)

    answer = input("\nIs the gripper around the cup body? Type yes to close: ").strip().lower()
    if answer != "yes":
        print("Stopped before closing.")
        return

    # 6. Close gripper
    set_gripper(mc, GRIPPER_CLOSED, "Closing")

    answer = input("\nLift cup? Type yes to lift: ").strip().lower()
    if answer != "yes":
        print("Stopped after gripping.")
        return

    # 7. Lift
    move_joints(mc, "LIFT_JOINTS", LIFT_JOINTS)

    print("\nDone.")


if __name__ == "__main__":
    main()