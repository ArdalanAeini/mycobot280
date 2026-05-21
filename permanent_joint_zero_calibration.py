"""
Permanent joint zero calibration for JetCobot / myCobot 6-axis.

WARNING:
  This script changes the robot's internal joint zero references.

Use only when:
  - The robot is safely supported
  - You can manually align each joint to the true mechanical zero position
  - You understand that incorrect calibration will make future motion inaccurate

Process:
  1. Connect to robot
  2. For each joint J1-J6:
      - release that joint
      - manually move it to physical zero
      - save current position as that joint's zero
  3. Test by moving to [0, 0, 0, 0, 0, 0]
"""

import time
import glob
from pymycobot.mycobot import MyCobot


BAUD = 1000000
SPEED = 10
WAIT_SEC = 3.0


def find_robot_port():
    ports = sorted(glob.glob("/dev/ttyUSB*"))

    if not ports:
        raise RuntimeError("No /dev/ttyUSB* device found. Check USB connection.")

    print("Available USB ports:", ports)
    return ports[0]


def report_angles(mc, label):
    angles = mc.get_angles()

    print("\n" + "=" * 50)
    print(label)
    print("=" * 50)

    if angles is None or len(angles) == 0:
        print("Could not read joint angles.")
    else:
        print("Current joint angles [deg]:", [round(a, 2) for a in angles])


def release_one_joint(mc, joint_id):
    """
    joint_id is 1 to 6.
    """
    print(f"\nReleasing joint J{joint_id}...")

    try:
        mc.release_servo(joint_id)
        time.sleep(1.0)
        print(f"J{joint_id} released.")
    except Exception as e:
        print(f"Could not release J{joint_id}: {e}")
        print("You may need to use release_all_servos() manually instead.")


def power_on_all_servos(mc):
    print("\nPowering on all servos...")

    try:
        mc.power_on()
        time.sleep(1.0)
        print("Servos powered on.")
    except Exception as e:
        print(f"Could not power on using mc.power_on(): {e}")
        print("If needed, restart the robot after calibration.")


def calibrate_joint_zero(mc, joint_id):
    print("\n" + "-" * 60)
    print(f"CALIBRATING J{joint_id}")
    print("-" * 60)

    print(f"Step A: I will release J{joint_id}.")
    print(f"Step B: You manually move J{joint_id} to its TRUE physical zero.")
    print(f"Step C: Press ENTER, and I will save this position as J{joint_id} = 0 degrees.")
    print()
    print("Be careful: once saved, this changes the robot's internal zero reference.")

    input(f"Press ENTER to release J{joint_id}...")

    release_one_joint(mc, joint_id)

    print()
    print(f"Now manually align J{joint_id} to its physical zero mark.")
    print("Do not force the robot hard. Move it gently.")
    print("When it is perfectly aligned, press ENTER.")

    input(f"Press ENTER to SAVE current J{joint_id} position as zero...")

    print(f"Saving current position as zero for J{joint_id}...")

    result = mc.set_servo_calibration(joint_id)
    time.sleep(1.0)

    print(f"Calibration result for J{joint_id}: {result}")

    report_angles(mc, f"After calibrating J{joint_id}")

    print(f"J{joint_id} zero calibration step complete.")


def test_zero_pose(mc):
    print("\n" + "=" * 60)
    print("ZERO POSE TEST")
    print("=" * 60)

    print("The robot will now move slowly to:")
    print("[0, 0, 0, 0, 0, 0]")
    print()
    print("Keep your hand near power/emergency stop.")
    print("Make sure the robot has clearance.")

    answer = input("Type yes to move to zero pose: ").strip().lower()

    if answer != "yes":
        print("Skipped zero pose test.")
        return

    mc.send_angles([0, 0, 0, 0, 0, 0], SPEED)
    time.sleep(WAIT_SEC)

    report_angles(mc, "After moving to [0, 0, 0, 0, 0, 0]")

    coords = mc.get_coords()
    if coords is not None and len(coords) > 0:
        print("Current coords:", [round(c, 2) for c in coords])

    print()
    print("Visually check:")
    print("Does the robot now physically match the true zero pose?")


def main():
    print("Permanent Joint Zero Calibration")
    print("================================")
    print()
    print("WARNING:")
    print("This script permanently changes joint zero references.")
    print("Wrong alignment will make robot motion inaccurate.")
    print()

    confirm = input("Type I UNDERSTAND to continue: ").strip()

    if confirm != "I UNDERSTAND":
        print("Cancelled.")
        return

    port = find_robot_port()
    print("Using port:", port)

    mc = MyCobot(port, BAUD)
    time.sleep(1.0)

    report_angles(mc, "Initial joint readings")

    print("\nCalibration order: J1, J2, J3, J4, J5, J6")
    print("You can skip any joint if it already looks correct.")
    print()

    for joint_id in range(1, 7):
        answer = input(f"Calibrate J{joint_id}? Type yes to calibrate, or press ENTER to skip: ").strip().lower()

        if answer == "yes":
            calibrate_joint_zero(mc, joint_id)
        else:
            print(f"Skipped J{joint_id}.")

    power_on_all_servos(mc)

    test_zero_pose(mc)

    print("\nCalibration script finished.")
    print("If zero pose looks correct, your joint zero calibration is now updated.")


if __name__ == "__main__":
    main()