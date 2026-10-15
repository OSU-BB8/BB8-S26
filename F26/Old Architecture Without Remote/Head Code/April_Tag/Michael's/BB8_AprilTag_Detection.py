
import cv2
import numpy as np
import math
import time
import os

from picamera2 import Picamera2
from pupil_apriltags import Detector

# ==========================================
# CONFIGURATION
# ==========================================

CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

# Actual width of the black AprilTag square
# (not including the surrounding white margin)
TAG_SIZE_METERS = 0.1524  # 6 inches

TAG_FAMILY = "tag36h11"

# Approximate camera horizontal field of view
# Replace with actual camera calibration later
CAMERA_HFOV = 62.0

# Show live camera window
SHOW_VIDEO = bool(os.environ.get("DISPLAY"))

# Terminal update interval
PRINT_INTERVAL = 0.2

# Detection quality/performance
DETECTION_DECIMATE = 1.0
DETECTION_THREADS = 4

METERS_TO_FEET = 3.28084

# ==========================================
# CAMERA SETUP
# ==========================================

print("Starting BB-8 AprilTag Detector...")

picam2 = Picamera2()

config = picam2.create_video_configuration(
    main={
        "size": (CAMERA_WIDTH, CAMERA_HEIGHT),
        "format": "RGB888"
    }
)

picam2.configure(config)
picam2.start()

time.sleep(2)

# ==========================================
# CAMERA PARAMETERS
# ==========================================

# Approximate focal length from horizontal FOV.
# Proper camera calibration will improve accuracy.

fx = CAMERA_WIDTH / (
    2 * math.tan(math.radians(CAMERA_HFOV / 2))
)

fy = fx
cx = CAMERA_WIDTH / 2
cy = CAMERA_HEIGHT / 2

camera_params = (fx, fy, cx, cy)

# ==========================================
# APRILTAG DETECTOR
# ==========================================

detector = Detector(
    families=TAG_FAMILY,
    nthreads=DETECTION_THREADS,
    quad_decimate=DETECTION_DECIMATE,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25
)

print("Camera started.")
print("AprilTag detection running...")
print("Press Q to quit.")
print("------------------------------------")

last_print = 0

# ==========================================
# MAIN LOOP
# ==========================================

try:
    while True:

        frame = picam2.capture_array()

        # Picamera2 RGB888 frames are arranged
        # as BGR for OpenCV in this configuration.
        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        # Detect all visible AprilTags
        tags = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=TAG_SIZE_METERS
        )

        results = []

        for tag in tags:

            tag_id = tag.tag_id

            # Position relative to camera
            x = float(tag.pose_t[0][0])
            y = float(tag.pose_t[1][0])
            z = float(tag.pose_t[2][0])

            # Straight-line distance
            distance_m = math.sqrt(
                x*x + y*y + z*z
            )

            distance_ft = (
                distance_m * METERS_TO_FEET
            )

            # Horizontal angle:
            # negative = left
            # positive = right
            angle_deg = math.degrees(
                math.atan2(x, z)
            )

            # Forward distance
            forward_ft = z * METERS_TO_FEET

            results.append({
                "id": tag_id,
                "distance_ft": distance_ft,
                "angle_deg": angle_deg,
                "forward_ft": forward_ft
            })

            # ==================================
            # DRAW DETECTION
            # ==================================

            corners = np.int32(tag.corners)

            cv2.polylines(
                frame,
                [corners],
                True,
                (0, 255, 0),
                2
            )

            center = tuple(
                np.int32(tag.center)
            )

            cv2.circle(
                frame,
                center,
                5,
                (0, 0, 255),
                -1
            )

            label = (
                f"ID:{tag_id} "
                f"D:{distance_ft:.2f}ft "
                f"A:{angle_deg:+.1f}deg"
            )

            text_x = max(
                0,
                int(np.min(corners[:, 0]))
            )

            text_y = max(
                25,
                int(np.min(corners[:, 1])) - 10
            )

            cv2.putText(
                frame,
                label,
                (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2
            )

        # ======================================
        # TERMINAL OUTPUT
        # ======================================

        now = time.monotonic()

        if now - last_print >= PRINT_INTERVAL:

            if results:

                print(
                    f"\nDetected {len(results)} tag(s)"
                )

                for result in results:

                    print(
                        f"Tag {result['id']:3d} | "
                        f"Distance: "
                        f"{result['distance_ft']:6.2f} ft | "
                        f"Angle: "
                        f"{result['angle_deg']:+6.1f} deg | "
                        f"Forward: "
                        f"{result['forward_ft']:6.2f} ft"
                    )

            else:
                print("No AprilTags detected")

            last_print = now

        # ======================================
        # VIDEO DISPLAY
        # ======================================

        if SHOW_VIDEO:

            cv2.putText(
                frame,
                f"Tags detected: {len(tags)}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2
            )

            cv2.imshow(
                "BB-8 AprilTag Detection",
                frame
            )

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

except KeyboardInterrupt:
    print("\nStopping detector...")

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    print("Camera stopped.")
