"""
Top-down hardcoded cup pickup test.

Sequence:
  1. Open gripper
  2. Move to HOME
  3. Move above the cup
  4. Move vertically down to the taught pickup position
  5. Close gripper
  6. Lift vertically back up

This avoids approaching the cup from the side.
"""

import time
import glob
from pymycobot.mycobot import MyCobot


BAUD = 1000000

ARM_SPEED = 15
DESCEND_SPEED = 8
GRIPPER_SPEED = 50

MODE = 1

GRIPPER_OPEN = 100
GRIPPER_CLOSED = 0

WAIT_SEC = 4.0
DESCEND_WAIT_SEC = 4.0
GRIPPER_WAIT_SEC = 1.5

# Safe home / park pose.
HOME_JOINTS = [0, 0, 0, 0, 90, 0]

# Your taught pickup position.
# This is the exact pose where the gripper grabs the cup.
PICK_COORDS = [223.9, -88.9, 20.1, 89.91, -1.58, 162.17]

# Same X/Y/orientation as pickup, but higher Z.
# This puts the gripper above the cup before descending vertically.
ABOVE_CUP_Z = 100.0
ABOVE_CUP_COORDS = [
    PICK_COORDS[0],
    PICK_COORDS[1],
    ABOVE_CUP_Z,
    PICK_COORDS[3],
    PICK_COORDS[4],
    PICK_COORDS[5],
]

# After gripping, lift straight back up.
LIFT_COORDS = ABOVE_CUP_COORDS.copy()


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


def move_joints(mc, label, joints, speed=ARM_SPEED):
    print(f"\nMoving to {label}: {joints}")
    mc.send_angles(joints, speed)
    time.sleep(WAIT_SEC)
    report_state(mc, f"After {label}")


def move_coords(mc, label, coords, speed=ARM_SPEED, wait_sec=WAIT_SEC):
    print(f"\nMoving to {label}: {coords}")
    mc.send_coords(coords, speed, MODE)
    time.sleep(wait_sec)
    report_state(mc, f"After {label}")


def main():
    print("Connecting to robot...")
    port = find_robot_port()
    print("Using port:", port)

    mc = MyCobot(port, BAUD)
    time.sleep(1.0)

    report_state(mc, "Initial state")

    print("\nThis script approaches the cup from above, then descends vertically.")
    print("Keep your hand near power/emergency stop.")
    input("Press ENTER to start...")

    # Step 1: open gripper
    set_gripper(mc, GRIPPER_OPEN, "Opening")

    # Step 2: go home first
    move_joints(mc, "HOME_JOINTS", HOME_JOINTS)

    # Step 3: move above cup
    move_coords(mc, "ABOVE_CUP_COORDS", ABOVE_CUP_COORDS, ARM_SPEED, WAIT_SEC)

    # Step 4: descend vertically to pickup pose
    answer = input("\nDescend vertically to cup body? Type yes: ").strip().lower()
    if answer == "yes":
        move_coords(mc, "PICK_COORDS", PICK_COORDS, DESCEND_SPEED, DESCEND_WAIT_SEC)
    else:
        print("Stopped above cup.")
        return

    # Step 5: close gripper
    answer = input("\nClose gripper to grab cup? Type yes: ").strip().lower()
    if answer == "yes":
        set_gripper(mc, GRIPPER_CLOSED, "Closing")
    else:
        print("Skipped gripper close.")
        return

    # Step 6: lift straight up
    answer = input("\nLift cup straight up? Type yes: ").strip().lower()
    if answer == "yes":
        move_coords(mc, "LIFT_COORDS", LIFT_COORDS, DESCEND_SPEED, DESCEND_WAIT_SEC)
    else:
        print("Skipped lift.")

    print("\nDone.")


if __name__ == "__main__":
    main()