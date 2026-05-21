"""
Top-down hardcoded cup pickup test.

Goal:
  1. Open gripper
  2. Go to HOME
  3. Move above the cup
  4. Descend vertically onto the cup body
  5. Close gripper
  6. Lift vertically

Important:
  - HOME -> ABOVE_CUP uses angular mode because it is easier for the robot to reach.
  - ABOVE_CUP -> PICK uses linear mode so it descends vertically.
"""

import time
import glob
from pymycobot.mycobot import MyCobot


BAUD = 1000000

ARM_SPEED = 15
DESCEND_SPEED = 8
GRIPPER_SPEED = 50

MODE_ANGULAR = 0
MODE_LINEAR = 1

GRIPPER_OPEN = 100
GRIPPER_CLOSED = 0

WAIT_SEC = 5.0
DESCEND_WAIT_SEC = 5.0
GRIPPER_WAIT_SEC = 1.5

# Safe home / park pose
HOME_JOINTS = [0, 0, 0, 0, 90, 0]

# Your taught pickup position from manual teaching.
# This is where the gripper should close on the cup.
PICK_COORDS = [223.9, -88.9, 20.1, 89.91, -1.58, 162.17]

# Same X/Y/orientation as pickup, but higher Z.
# If this is too low/high, change only this number first.
ABOVE_CUP_Z = 100.0

ABOVE_CUP_COORDS = [
    PICK_COORDS[0],
    PICK_COORDS[1],
    ABOVE_CUP_Z,
    PICK_COORDS[3],
    PICK_COORDS[4],
    PICK_COORDS[5],
]

LIFT_COORDS = ABOVE_CUP_COORDS.copy()


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
        current_value = mc.get_gripper_value()
        print("Gripper value:", current_value)
    except Exception:
        print("Could not read gripper value.")


def move_joints(mc, label, joints, speed=ARM_SPEED, wait_sec=WAIT_SEC):
    print(f"\n--- Moving to {label} ---")
    print("Target joints:", joints)

    mc.send_angles(joints, speed)
    time.sleep(wait_sec)

    report_state(mc, f"After {label}")


def move_coords(mc, label, coords, speed, mode, wait_sec):
    print(f"\n--- Moving to {label} ---")
    print("Target coords:", coords)
    print("Speed:", speed)
    print("Mode:", mode)

    mc.send_coords(coords, speed, mode)
    time.sleep(wait_sec)

    report_state(mc, f"After {label}")


def main():
    print("Connecting to robot...")
    port = find_robot_port()
    print("Using port:", port)

    mc = MyCobot(port, BAUD)
    time.sleep(1.0)

    report_state(mc, "Initial state")

    print("\nThis script will do a top-down pickup test.")
    print("Sequence:")
    print("  1. Open gripper")
    print("  2. Go HOME")
    print("  3. Move above cup")
    print("  4. Descend vertically")
    print("  5. Close gripper")
    print("  6. Lift vertically")
    print("\nKeep your hand near the power/emergency stop.")
    input("Press ENTER to start...")

    # Step 1: open gripper
    set_gripper(mc, GRIPPER_OPEN, "Opening")

    # Step 2: go home
    move_joints(mc, "HOME_JOINTS", HOME_JOINTS)

    # Step 3: move above the cup
    # Angular mode is used here because moving from home to cup may not be possible as a straight line.
    move_coords(
        mc,
        "ABOVE_CUP_COORDS",
        ABOVE_CUP_COORDS,
        ARM_SPEED,
        MODE_ANGULAR,
        WAIT_SEC,
    )

    answer = input("\nDid the gripper move above the cup? Type yes to continue: ").strip().lower()
    if answer != "yes":
        print("Stopped. Adjust ABOVE_CUP_Z or PICK_COORDS before trying again.")
        return

    # Step 4: descend vertically
    move_coords(
        mc,
        "PICK_COORDS",
        PICK_COORDS,
        DESCEND_SPEED,
        MODE_LINEAR,
        DESCEND_WAIT_SEC,
    )

    answer = input("\nIs the gripper correctly around the cup body? Type yes to close: ").strip().lower()
    if answer != "yes":
        print("Stopped before closing gripper.")
        return

    # Step 5: close gripper
    set_gripper(mc, GRIPPER_CLOSED, "Closing")

    answer = input("\nLift the cup vertically? Type yes to lift: ").strip().lower()
    if answer != "yes":
        print("Stopped after gripping.")
        return

    # Step 6: lift vertically
    move_coords(
        mc,
        "LIFT_COORDS",
        LIFT_COORDS,
        DESCEND_SPEED,
        MODE_LINEAR,
        DESCEND_WAIT_SEC,
    )

    print("\nDone.")


if __name__ == "__main__":
    main()