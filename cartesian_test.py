"""
Cartesian motion test - milestone 2 + gripper integration.

Mime version of pick-and-place:
  1. Open gripper, move to HOME_UP
  2. Move to LEFT  (the "pick" position)
  3. CLOSE gripper (mime grab)
  4. Move to RIGHT (the "place" position)
  5. OPEN gripper (mime release)
  6. Back to HOME_UP
  7. Park with gripper open

End state: arm straight up, gripper horizontal, gripper open.

Gripper:
  - Open  = value 100 (~45 mm jaw spacing)
  - Closed = value 0   (~20 mm jaw spacing)
  - The gripper API is independent of the arm motion - we just call
    mc.set_gripper_value() whenever we want it to open/close.
"""

import time
from pymycobot.mycobot import MyCobot


PORT = "/dev/ttyUSB0"
BAUD = 1000000

SPEED = 30          # arm speed
GRIPPER_SPEED = 50  # gripper speed
MODE = 1
WAIT_SEC = 5.0
GRIPPER_WAIT_SEC = 2.0

# Gripper orientation: pointing down with preferred jaw angle.
# Discovered via wrist_orient_test.py.
RX_DOWN = 178.4
RY_DOWN = -1.6
RZ_DOWN = -133.4

# Gripper values (0 = fully closed, 100 = fully open).
GRIPPER_OPEN  = 100
GRIPPER_CLOSED = 0

# Test poses [X mm, Y mm, Z mm, RX deg, RY deg, RZ deg]
POSE_HOME_UP   = [100,    0, 280, RX_DOWN, RY_DOWN, RZ_DOWN]
POSE_LEFT      = [100,  100, 200, RX_DOWN, RY_DOWN, RZ_DOWN]
POSE_RIGHT     = [100, -100, 200, RX_DOWN, RY_DOWN, RZ_DOWN]

# Final park pose: arm straight up, gripper horizontal.
PARK_JOINTS = [20, 20, 20, 20, 20, 20]


def report_position(mc, label):
    coords = mc.get_coords()
    if coords is None or len(coords) == 0:
        print(f"  >> {label}: <couldn't read coords>")
        return
    x, y, z, rx, ry, rz = coords
    print(
        f"  >> {label}: "
        f"X={x:+7.1f}  Y={y:+7.1f}  Z={z:+7.1f}  "
        f"RX={rx:+6.1f}  RY={ry:+6.1f}  RZ={rz:+6.1f}  (mm/deg)"
    )


def report_gripper(mc):
    val = mc.get_gripper_value()
    print(f"     gripper value: {val}  (0=closed, 100=open)")


def move_to_pose(mc, label, pose):
    print(f"\n--- Moving to {label}: {pose} ---")
    mc.send_coords(pose, SPEED, MODE)
    time.sleep(WAIT_SEC)
    report_position(mc, f"after {label}")
    angles = mc.get_angles()
    if angles is not None and len(angles) > 0:
        rounded = [round(a, 1) for a in angles]
        print(f"     joints (deg): {rounded}")


def set_gripper(mc, value, label):
    print(f"\n--- {label} gripper (value={value}) ---")
    mc.set_gripper_value(value, GRIPPER_SPEED)
    time.sleep(GRIPPER_WAIT_SEC)
    report_gripper(mc)


def main():
    print("Connecting to arm...")
    mc = MyCobot(PORT, BAUD)
    time.sleep(0.5)

    print("\n[Initial state]")
    report_position(mc, "start")
    report_gripper(mc)

    # Step 1: Start with gripper open, move to home above the workspace.
    set_gripper(mc, GRIPPER_OPEN, "Opening")
    move_to_pose(mc, "HOME_UP", POSE_HOME_UP)

    # Step 2: Move to the "pick" position.
    move_to_pose(mc, "LEFT (pick position)", POSE_LEFT)

    # Step 3: Mime the grab.
    set_gripper(mc, GRIPPER_CLOSED, "CLOSING (mime grab)")

    # Step 4: Transport to the "place" position.
    move_to_pose(mc, "RIGHT (place position)", POSE_RIGHT)

    # Step 5: Mime the release.
    set_gripper(mc, GRIPPER_OPEN, "OPENING (mime release)")

    # Step 6: Back to home.
    move_to_pose(mc, "HOME_UP", POSE_HOME_UP)

    # Step 7: Park - arm straight up, gripper horizontal, gripper open.
    print(f"\n[Parking arm: straight up, gripper horizontal]")
    mc.send_angles(PARK_JOINTS, SPEED)
    time.sleep(WAIT_SEC)
    final_angles = mc.get_angles()
    print(f"  Final joints (deg): {[round(a, 1) for a in final_angles]}")
    report_position(mc, "final pose")
    report_gripper(mc)
    print("Done.")


if __name__ == "__main__":
    main()