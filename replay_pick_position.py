"""
Replay taught pickup position with gripper rotation first.

Sequence:
  1. Open gripper
  2. Move to HOME
  3. Rotate gripper/wrist while still away from the cup
  4. Move near the cup
  5. Move into exact taught pickup pose
  6. Close gripper
  7. Lift cup
"""

import time
import glob
from pymycobot.mycobot import MyCobot


BAUD = 1000000

ARM_SPEED = 15
GRIPPER_SPEED = 50

GRIPPER_OPEN = 100
GRIPPER_CLOSED = 0

WAIT_SEC = 4.0
GRIPPER_WAIT_SEC = 1.5

# Your taught pickup pose from the manual teaching step
PICK_JOINTS = [-17.84, -125.24, -0.79, 43.15, 90.08, 98.7]

# Safe home pose
HOME_JOINTS = [0, 0, 0, 0, 90, 0]

# Same as HOME, but rotate J6 first to match the pickup gripper rotation.
# This changes the gripper angle before going near the cup.
ROTATED_HOME_JOINTS = [0, 0, 0, 0, 90, 98.7]

# Approach pose:
# Same as pickup pose, but J2 is less negative, so the gripper should be higher/safer.
# This gets close to the cup before final descent.
APPROACH_JOINTS = [-17.84, -105.24, -0.79, 43.15, 90.08, 98.7]

# Final lift after grabbing.
# Same as approach pose for now.
LIFT_JOINTS = [-17.84, -105.24, -0.79, 43.15, 90.08, 98.7]


def find_robot_port():
    ports = sorted(glob.glob("/dev/ttyUSB*"))
    if not ports:
        raise RuntimeError("No /dev/ttyUSB* device found. Check robot power and USB cable.")
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
    else:
        print("Joints: <could not read>")

    if coords:
        print("Coords:", [round(c, 2) for c in coords])
    else:
        print("Coords: <could not read>")


def set_gripper(mc, value, label):
    print(f"\n{label} gripper: value={value}")
    mc.set_gripper_value(value, GRIPPER_SPEED)
    time.sleep(GRIPPER_WAIT_SEC)


def move_joints(mc, label, joints):
    print(f"\nMoving to {label}: {joints}")
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

    print("\nThis script rotates the gripper first, then approaches the cup.")
    print("Keep your hand near power/emergency stop.")
    input("Press ENTER to start...")

    # Step 1: open gripper
    set_gripper(mc, GRIPPER_OPEN, "Opening")

    # Step 2: move to safe home
    move_joints(mc, "HOME_JOINTS", HOME_JOINTS)

    # Step 3: rotate gripper first, while away from cup
    move_joints(mc, "ROTATED_HOME_JOINTS", ROTATED_HOME_JOINTS)

    # Step 4: move close to the cup, but not all the way down
    move_joints(mc, "APPROACH_JOINTS", APPROACH_JOINTS)

    # Step 5: final move into exact taught pickup position
    answer = input("\nMove into exact PICK_JOINTS position? Type yes: ").strip().lower()
    if answer == "yes":
        move_joints(mc, "PICK_JOINTS", PICK_JOINTS)
    else:
        print("Stopped before pickup position.")
        return

    # Step 6: close gripper
    answer = input("\nClose gripper to grab the cup? Type yes: ").strip().lower()
    if answer == "yes":
        set_gripper(mc, GRIPPER_CLOSED, "Closing")
    else:
        print("Skipped gripper close.")
        return

    # Step 7: lift cup
    answer = input("\nLift the cup? Type yes: ").strip().lower()
    if answer == "yes":
        move_joints(mc, "LIFT_JOINTS", LIFT_JOINTS)
    else:
        print("Skipped lift.")

    print("\nDone.")


if __name__ == "__main__":
    main()