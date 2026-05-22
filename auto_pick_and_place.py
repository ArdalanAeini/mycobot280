"""
Auto pick-and-place with GRIPPER FLIP at home.

Why the flip:
  The gripper has a servo housing on its underside. With the gripper in
  its "natural" orientation, that housing blocks the gripper from
  reaching small/short objects close to the desk surface.
  Solution: rotate J6 by -180 degrees at home BEFORE approaching the cup.
  Now the slim side is down and the gripper can reach small objects.

Strategy:
  - Joint replay for the precise grab approach (taught with the flip)
  - Cartesian motion with orientation LOCKED for lift / swing / lower
    (keeps the gripper horizontal -- water in the cup wouldn't spill)
  - Place position = pick position mirrored across Y axis

The flip + un-flip wraps around the approach so the arm returns to its
natural home pose at the end.
"""

import time
import glob
from pymycobot.mycobot import MyCobot


BAUD = 1000000

ARM_SPEED = 25
GRIPPER_SPEED = 50
GRIPPER_OPEN = 100
GRIPPER_CLOSED = 0

WAIT_SEC = 4.0
GRIPPER_WAIT_SEC = 1.5

# Cartesian motion mode: 1 = linear (straight-line in 3D space).
CARTESIAN_MODE = 1

# How much to lift the cup vertically (mm). 80 mm clears most desk objects.
LIFT_HEIGHT_MM = 80

# Home poses.
#   HOME            -- arm straight up, gripper in its natural orientation.
#   HOME_FLIPPED    -- same as HOME but J6 rotated -180 so the gripper's
#                      slim side faces down. This is the pose we sit in
#                      BEFORE moving to PICK_ABOVE.
HOME_JOINTS         = [0, 0, 0, 0, 90,    0]
HOME_FLIPPED_JOINTS = [0, 0, 0, 0, 0, +180]

# Taught positions from teach_pickup_pair.py.
# These were taught with the gripper already in the FLIPPED orientation.
PICK_ABOVE_JOINTS = [-21.09, -97.20, -0.79, -88.41, 105.90, -10.10]
PICK_GRAB_JOINTS  = [-15.02, -97.29, -0.79, -84.46, 109.59,  -5.00]

# Cartesian coords for PICK_GRAB. Used as the starting point for the
# Cartesian lift / swing / lower calculations.
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


def move_joints(mc, label, joints):
    print(f"\nMoving (joint) to {label}: {joints}")
    mc.send_angles(joints, ARM_SPEED)
    time.sleep(WAIT_SEC)
    report_state(mc, f"After {label}")


def move_cartesian(mc, label, coords):
    print(f"\nMoving (Cartesian) to {label}: {coords}")
    mc.send_coords(coords, ARM_SPEED, CARTESIAN_MODE)
    time.sleep(WAIT_SEC)
    report_state(mc, f"After {label}")


def main():
    print("Connecting to robot...")
    port = find_robot_port()
    print("Using port:", port)
    mc = MyCobot(port, BAUD)
    time.sleep(1.0)

    report_state(mc, "Initial state")

    # Compute derived Cartesian waypoints.
    # Lift = same XY and orientation as PICK_GRAB, Z raised by LIFT_HEIGHT_MM
    pick_lift_coords = list(PICK_GRAB_COORDS)
    pick_lift_coords[2] += LIFT_HEIGHT_MM

    # Place lift = pick lift mirrored across Y (left-right swap)
    place_lift_coords = list(pick_lift_coords)
    place_lift_coords[1] = -place_lift_coords[1]

    # Place grab = mirror Y of PICK_GRAB_COORDS
    place_grab_coords = list(PICK_GRAB_COORDS)
    place_grab_coords[1] = -place_grab_coords[1]

    # Place release-and-back-away = place grab with Z lifted
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
    move_joints(mc, "HOME", HOME_JOINTS)

    # 3. Flip the gripper 180 degrees so the slim side faces down.
    #    This gives clearance to reach the small cup on the desk.
    move_joints(mc, "HOME_FLIPPED (J6 = -180)", HOME_FLIPPED_JOINTS)

    # 4. Move to the taught approach position
    move_joints(mc, "PICK_ABOVE", PICK_ABOVE_JOINTS)

    # 5. Sweep into the taught grab position
    move_joints(mc, "PICK_GRAB", PICK_GRAB_JOINTS)

    # 6. Close gripper around the cup
    set_gripper(mc, GRIPPER_CLOSED, "Closing")

    # 7. LIFT vertically. Orientation LOCKED -> gripper stays horizontal,
    #    water in the cup wouldn't spill.
    move_cartesian(mc, "PICK_LIFT (vertical up)", pick_lift_coords)

    # 8. SWING horizontally to mirror Y. Orientation locked.
    move_cartesian(mc, "PLACE_LIFT (horizontal swing)", place_lift_coords)

    # 9. LOWER vertically. Orientation locked.
    move_cartesian(mc, "PLACE_GRAB (vertical down)", place_grab_coords)

    # 10. Release the cup
    set_gripper(mc, GRIPPER_OPEN, "Opening / releasing")

    # 11. Lift away from the placed cup. Orientation locked.
    move_cartesian(mc, "PLACE_RELEASE (vertical up)", place_release_coords)

    # 12. Un-flip the gripper before going home (J6 back to 0).
    move_joints(mc, "HOME_FLIPPED (return through flipped pose)", HOME_FLIPPED_JOINTS)

    # 13. Return HOME (natural orientation)
    move_joints(mc, "HOME", HOME_JOINTS)

    print("\n" + "=" * 50)
    print("Pick-and-place complete.")
    print("=" * 50)


if __name__ == "__main__":
    main()