import cv2
import numpy as np
import math
import time
import os
import socket
from pathlib import Path

from picamera2 import Picamera2
from pupil_apriltags import Detector
import pygame

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
SHOW_VIDEO = os.environ.get("BB8_SHOW_VIDEO", "0") == "1" and bool(os.environ.get("DISPLAY"))

# Terminal update interval
PRINT_INTERVAL = 0.2

# Detection quality/performance
DETECTION_DECIMATE = 1.0
DETECTION_THREADS = 4

METERS_TO_FEET = 3.28084

# Only tag 1 controls movement. Other IDs remain visible in logs.
FOLLOW_TAG_ID = 1
BODY_PI_IP = os.environ.get("BB8_BODY_IP", "10.42.0.27")
BODY_TARGET_PORT = 5005
SEND_PERIOD = 0.1  # 10 Hz, faster than Body Pi 0.5s target timeout
MAX_DISTANCE_FT = 50.0
MAX_ABS_ANGLE_DEG = 90.0
# No stale target holding: no ID 1 detection -> no UDP packets.

# ==========================================
# AUDIO CONFIGURATION
# ==========================================

# Audio files go in an "audio" folder beside this script.
# Example: Head Code/audio/audio0.mp3
SCRIPT_DIR = Path(__file__).resolve().parent
AUDIO_DIR = SCRIPT_DIR / "audio"

# AprilTag ID -> audio file. Tag 0 plays audio0.mp3.
TAG_AUDIO = {
    0: AUDIO_DIR / "audio0.mp3",
}

# Don't repeatedly trigger while the same tag stays visible.
AUDIO_COOLDOWN_SECONDS = 3.0

# ==========================================
# AUDIO SETUP
# ==========================================

audio_ready = False
try:
    pygame.mixer.init()
    audio_ready = True
    for tag_id, audio_path in TAG_AUDIO.items():
        if not audio_path.is_file():
            print(
                f"[AUDIO] Missing file for tag {tag_id}: {audio_path}. "
                "Detection will continue, but this tag cannot play audio.",
                flush=True
            )
except pygame.error as exc:
    print(f"[AUDIO] Audio initialization failed: {exc}", flush=True)

previous_tag_ids = set()
last_audio_time = {}

def trigger_audio_for_new_tags(current_tag_ids):
    """Play mapped audio once when a tag first appears in view."""
    if not audio_ready:
        return

    now = time.monotonic()
    newly_seen_ids = current_tag_ids - previous_tag_ids

    for tag_id in newly_seen_ids:
        audio_path = TAG_AUDIO.get(tag_id)
        if audio_path is None:
            continue

        if not audio_path.is_file():
            print(f"[AUDIO] File not found: {audio_path}", flush=True)
            continue

        if now - last_audio_time.get(tag_id, -1e9) < AUDIO_COOLDOWN_SECONDS:
            continue

        try:
            # Do not interrupt an audio clip that is already playing.
            if pygame.mixer.music.get_busy():
                print(f"[AUDIO] Skipping tag {tag_id}; audio is already playing.", flush=True)
                continue

            pygame.mixer.music.load(str(audio_path))
            pygame.mixer.music.play()
            last_audio_time[tag_id] = now
            print(f"[AUDIO] AprilTag {tag_id} detected -> playing {audio_path.name}", flush=True)
        except pygame.error as exc:
            print(f"[AUDIO] Could not play {audio_path}: {exc}", flush=True)

# ==========================================
# CAMERA SETUP
# ==========================================

print("Starting BB-8 AprilTag Detector + Audio...")

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
print("Tag 0 plays audio/audio0.mp3 when it first appears.")
print("Only Tag 1 controls movement.")
print("Press Q to quit when video is enabled.")
print("------------------------------------")

last_print = 0
last_send = 0.0
target_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ==========================================
# MAIN LOOP
# ==========================================

try:
    while True:

        frame = picam2.capture_array()

        # Preserve the original project's frame conversion.
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
        current_tag_ids = set()

        for tag in tags:

            tag_id = int(tag.tag_id)
            current_tag_ids.add(tag_id)

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
        # AUDIO: TAG 0 -> audio/audio0.mp3
        # ======================================
        # Uses the exact same detections as movement logic.
        # Plays once when a mapped tag first enters view.
        trigger_audio_for_new_tags(current_tag_ids)
        previous_tag_ids = current_tag_ids.copy()

        # ======================================
        # SEND ONLY TAG 1 TO BODY PI
        # ======================================
        # Body Pi already understands distance_ft,angle_deg on UDP 5005.
        # If tag 1 disappears, stop sending: Body Pi times out in 0.5 s.
        # Other IDs are deliberately ignored for motion.
        now = time.monotonic()
        follow = next((r for r in results if r["id"] == FOLLOW_TAG_ID), None)
        if follow is not None and now - last_send >= SEND_PERIOD:
            distance = follow["distance_ft"]
            angle = follow["angle_deg"]
            if (math.isfinite(distance) and math.isfinite(angle)
                    and 0.0 <= distance <= MAX_DISTANCE_FT
                    and abs(angle) <= MAX_ABS_ANGLE_DEG):
                packet = f"{distance:.3f},{angle:.2f}"
                try:
                    target_sock.sendto(packet.encode("ascii"), (BODY_PI_IP, BODY_TARGET_PORT))
                except OSError as exc:
                    print(f"[APRILTAG] UDP send failed: {exc}", flush=True)
                last_send = now

        # ======================================
        # TERMINAL OUTPUT
        # ======================================

        now = time.monotonic()

        if now - last_print >= PRINT_INTERVAL:

            if results:

                print(
                    f"\\nDetected {len(results)} tag(s)"
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
    print("\\nStopping detector...")

finally:
    target_sock.close()
    picam2.stop()
    cv2.destroyAllWindows()
    if audio_ready:
        pygame.mixer.quit()
    print("Camera stopped.")
