"""
Teach pick position manually.

Purpose:
  Manually move the robot gripper to the exact cup pickup position,
  then record the joint angles.

Process:
  1. Connect to robot
  2. Open gripper
  3. Release all servos
  4. You manually move the gripper around the cup
  5. Press ENTER
  6. Robot powers servos back on
  7. Script records the joint angles and coordinates
  8. Saves them to taught_pick_position.txt

This is a hardcoded/manual pickup calibration.
"""

import time
import glob
from pymycobot.mycobot import MyCobot


BAUD = 1000000

GRIPPER_OPEN = 100
GRIPPER_CLOSED = 0
GRIPPER_SPEED = 50

ARM_SPEED = 20

OUTPUT_FILE = "taught_pick_position.txt"


def find_robot_port():
    ports = sorted(glob.glob("/dev/ttyUSB0"))
    if not ports:
        raise RuntimeError("No /dev/ttyUSB0 device found. Check robot power and USB cable.")
    print("Available USB ports:", ports)
    return ports[0]


def report_state(mc, label):
    print("\n" + "=" * 50)
    print(label)
    print("=" * 50)

    angles = mc.get_angles()
    coords = mc.get_coords()

    if angles is None or len(angles) == 0:
        print("Joints: <could not read>")
    else:
        print("Joints [deg]:", [round(a, 2) for a in angles])

    if coords is None or len(coords) == 0:
        print("Coords: <could not read>")
    else:
        print("Coords [mm/deg]:", [round(c, 2) for c in coords])

    return angles, coords


def open_gripper(mc):
    print("\nOpening gripper...")
    mc.set_gripper_value(GRIPPER_OPEN, GRIPPER_SPEED)
    time.sleep(1.5)


def close_gripper(mc):
    print("\nClosing gripper...")
    mc.set_gripper_value(GRIPPER_CLOSED, GRIPPER_SPEED)
    time.sleep(1.5)


def save_position(angles, coords):
    with open(OUTPUT_FILE, "w") as f:
        f.write("Taught pickup position\n")
        f.write("======================\n\n")

        if angles is not None and len(angles) > 0:
            rounded_angles = [round(a, 2) for a in angles]
            f.write(f"PICK_JOINTS = {rounded_angles}\n")

        if coords is not None and len(coords) > 0:
            rounded_coords = [round(c, 2) for c in coords]
            f.write(f"PICK_COORDS = {rounded_coords}\n")

    print(f"\nSaved taught position to: {OUTPUT_FILE}")


def main():
    print("Connecting to robot...")
    port = find_robot_port()
    print("Using port:", port)

    mc = MyCobot(port, BAUD)
    time.sleep(1.0)

    report_state(mc, "Initial state")

    print("\nStep 1: Opening gripper.")
    open_gripper(mc)

    print("\nStep 2: Releasing all servos.")
    print("After this, gently move the robot by hand.")
    input("Press ENTER to release all servos...")

    mc.release_all_servos()
    time.sleep(1.0)

    print("\nNow manually move the gripper around the cup.")
    print("Position it exactly where it should grab the cup.")
    print("Do not force the joints.")
    input("When the gripper is in the correct pickup position, press ENTER...")

    print("\nStep 3: Powering servos back on to hold position.")
    mc.power_on()
    time.sleep(1.0)

    print("\nStep 4: Reading taught pickup position.")
    angles, coords = report_state(mc, "Taught pickup position")

    save_position(angles, coords)

    print("\nOptional test:")
    print("The gripper can close now to test if this position grabs the cup.")
    answer = input("Close gripper now? Type yes to close: ").strip().lower()

    if answer == "yes":
        close_gripper(mc)
        report_state(mc, "After closing gripper")
    else:
        print("Skipped gripper close.")

    print("\nDone.")
    print("You can now use the saved PICK_JOINTS as your hardcoded pickup pose.")


if __name__ == "__main__":
    main()