"""
Camera test - milestone 3.

Goal: prove the wrist-mounted camera works.

What this does:
  1. Try to open /dev/video0 (and fallback to /dev/video1)
  2. Capture a single frame
  3. Print the frame's shape and basic stats
  4. Save it as a JPG file

After running, open the saved JPG to confirm what the camera sees.

Why the fallback: Jetson boards usually expose multiple /dev/video*
nodes, even if only one camera is connected. We try them in order
and use whichever one works.
"""

import cv2
import time
import os


CAMERA_CANDIDATES = ["/dev/video0", "/dev/video1"]

OUTPUT_DIR = os.path.expanduser("~/src/captures")
OUTPUT_FILE = os.path.join(OUTPUT_DIR, "wrist_camera_test.jpg")

# Cameras output garbage for the first few frames as auto-exposure
# stabilizes. We "burn" some frames before saving.
WARMUP_FRAMES = 10


def try_camera(device):
    """Try to open a camera. Returns cv2.VideoCapture if it works."""
    print(f"\nTrying {device}...")
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        print(f"  Could NOT open {device}")
        return None

    ret, frame = cap.read()
    if not ret or frame is None:
        print(f"  Opened {device} but couldn't grab a frame.")
        cap.release()
        return None

    print(f"  SUCCESS: opened {device}, frame shape: {frame.shape}")
    return cap


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # Find a working camera.
    cap = None
    used_device = None
    for device in CAMERA_CANDIDATES:
        cap = try_camera(device)
        if cap is not None:
            used_device = device
            break

    if cap is None:
        print("\nERROR: No working camera found.")
        print("Check: is the camera USB cable plugged in to the Jetson?")
        print("Also try: ls /dev/video*")
        return

    print(f"\nUsing camera: {used_device}")

    # Warm up.
    print(f"Warming up ({WARMUP_FRAMES} frames)...")
    for i in range(WARMUP_FRAMES):
        ret, _ = cap.read()
        time.sleep(0.05)

    # Grab the final frame.
    print("Capturing final frame...")
    ret, frame = cap.read()
    if not ret or frame is None:
        print("ERROR: couldn't grab final frame.")
        cap.release()
        return

    h, w, c = frame.shape
    avg_brightness = frame.mean()
    print(f"\n[Captured frame]")
    print(f"  Resolution: {w} x {h}  ({c} channels)")
    print(f"  Average brightness: {avg_brightness:.1f}  (0=black, 255=white)")
    if avg_brightness < 5:
        print("  WARNING: image is nearly black. Lens cap on? Camera blocked?")
    elif avg_brightness > 240:
        print("  WARNING: image is nearly white. Overexposed?")
    else:
        print("  Brightness looks reasonable.")

    cv2.imwrite(OUTPUT_FILE, frame)
    print(f"\nSaved to: {OUTPUT_FILE}")

    cap.release()
    print("\nDone.")


if __name__ == "__main__":
    main()
