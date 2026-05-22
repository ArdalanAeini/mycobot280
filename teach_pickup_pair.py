"""
Kinesthetic teaching for the pickup waypoint PAIR.

Records two positions:
  1. PICK_ABOVE  - gripper hovering above the cup, jaws open, ready to descend
  2. PICK_GRAB   - gripper lowered so jaws are around the cup body, ready to close

The auto pick-and-place script will then:
  - Move to PICK_ABOVE (safe approach above the cup)
  - Move straight to PICK_GRAB (the descent)
  - Close gripper
  - Lift back to PICK_ABOVE
  - ... transport to place ...

Both positions are saved to taught_pickup_pair.txt with full joint angles
AND Cartesian coords for documentation. The replay uses the joint values.

NOTE: We deliberately use joint replay (not Cartesian replay) here because
the user-taught positions captured the actual physical arm configuration
exactly. Cartesian replay might pick a different IK solution and arrive
at the same XYZ via a different arm pose, which could cause collisions.
"""

import time
import glob
from pymycobot.mycobot import MyCobot


BAUD = 1000000
GRIPPER_OPEN = 100
GRIPPER_SPEED = 50
OUTPUT_FILE = "taught_pickup_pair.txt"


def find_robot_port():
    ports = sorted(glob.glob("/dev/ttyUSB*"))
    if not ports:
        raise RuntimeError("No /dev/ttyUSB* device. Check robot power and USB cable.")
    print("Available USB ports:", ports)
    return ports[0]


def read_state(mc, label):
    print("\n" + "=" * 50)
    print(label)
    print("=" * 50)
    angles = mc.get_angles()
    coords = mc.get_coords()
    if angles is not None and len(angles) > 0:
        print("Joints [deg]:", [round(a, 2) for a in angles])
    else:
        print("Joints: <could not read>")
        angles = None
    if coords is not None and len(coords) > 0:
        print("Coords [mm/deg]:", [round(c, 2) for c in coords])
    else:
        print("Coords: <could not read>")
        coords = None
    return angles, coords


def open_gripper(mc):
    print("\nOpening gripper...")
    mc.set_gripper_value(GRIPPER_OPEN, GRIPPER_SPEED)
    time.sleep(1.5)


def save_pair(above_angles, above_coords, grab_angles, grab_coords):
    with open(OUTPUT_FILE, "w") as f:
        f.write("Taught pickup waypoint pair\n")
        f.write("===========================\n\n")

        if above_angles is not None:
            f.write(f"PICK_ABOVE_JOINTS = {[round(a, 2) for a in above_angles]}\n")
        if above_coords is not None:
            f.write(f"PICK_ABOVE_COORDS = {[round(c, 2) for c in above_coords]}\n")

        f.write("\n")

        if grab_angles is not None:
            f.write(f"PICK_GRAB_JOINTS = {[round(a, 2) for a in grab_angles]}\n")
        if grab_coords is not None:
            f.write(f"PICK_GRAB_COORDS = {[round(c, 2) for c in grab_coords]}\n")

    print(f"\nSaved both taught positions to: {OUTPUT_FILE}")


def main():
    print("Connecting to robot...")
    port = find_robot_port()
    print("Using port:", port)
    mc = MyCobot(port, BAUD)
    time.sleep(1.0)

    read_state(mc, "Initial state")

    # Step 1: Open gripper so you have clearance to position around the cup
    print("\nStep 1: Opening gripper.")
    open_gripper(mc)

    # Step 2: Release servos so the operator can manually move the arm
    print("\nStep 2: About to release all servos.")
    print("After this, the arm will go floppy. SUPPORT IT with your hand.")
    input("Press ENTER to release servos...")
    mc.release_all_servos()
    time.sleep(1.0)

    # Step 3: Capture PICK_ABOVE
    print("\nStep 3: Manually position the gripper ABOVE the cup.")
    print("  - Open jaws should straddle the cup top, with about 3-5 cm clearance")
    print("  - This is the SAFE APPROACH position")
    print("  - The robot will hover here before descending")
    input("\nWhen the gripper is correctly above the cup, press ENTER...")
    above_angles, above_coords = read_state(mc, "PICK_ABOVE captured")

    # Step 4: Capture PICK_GRAB
    print("\nStep 4: Now manually LOWER the gripper straight down.")
    print("  - The jaws should be around the cup body")
    print("  - This is the GRAB position - where gripper will close on the cup")
    input("\nWhen the gripper is correctly around the cup, press ENTER...")
    grab_angles, grab_coords = read_state(mc, "PICK_GRAB captured")

    # Step 5: Re-power servos so the arm holds its current pose
    print("\nStep 5: Powering servos back on to hold position.")
    mc.power_on()
    time.sleep(1.0)

    # Step 6: Save both
    save_pair(above_angles, above_coords, grab_angles, grab_coords)

    print("\n" + "=" * 50)
    print("Teaching complete.")
    print("=" * 50)
    print("The arm is now holding the PICK_GRAB pose.")
    print("Next step: run auto_pick_and_place.py to use these positions.")


if __name__ == "__main__":
    main()
    