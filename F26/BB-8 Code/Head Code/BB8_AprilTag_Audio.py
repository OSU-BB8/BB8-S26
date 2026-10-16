#!/usr/bin/env python3
"""
BB-8 F26 AprilTag -> audio add-on.

Install this alongside the other files in:
  F26/BB-8 Code/Head Code/

This is a standalone test/add-on: it opens the camera itself and plays audio
locally on the Raspberry Pi running this script. Stop any other program that
is currently using the same camera before starting it.

AprilTag family must match the tags printed for your project.
"""
from pathlib import Path
import time
import sys

import cv2
import pygame
from picamera2 import Picamera2
from pupil_apriltags import Detector

HERE = Path(__file__).resolve().parent
AUDIO_DIR = HERE / "audio"

#audio files
TAG_AUDIO = {
    0: AUDIO_DIR / "audio0.mp3",
    
}

TAG_FAMILY = "tag36h11"
COOLDOWN_SECONDS = 3.0

def main():
    missing = [str(path) for path in TAG_AUDIO.values() if not path.is_file()]
    if missing:
        print("Missing audio file(s):")
        for path in missing:
            print(f"  {path}")
        print(f"\nPut your sound files in: {AUDIO_DIR}")
        print("Or update TAG_AUDIO near the top of this script.")
        return 2

    pygame.mixer.init()
    detector = Detector(families=TAG_FAMILY)

    camera = Picamera2()
    camera.configure(camera.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    ))
    camera.start()
    time.sleep(1.0)

    last_triggered = {}
    last_seen = {}
    previous_ids = set()
    print("BB-8 AprilTag audio add-on running.")
    print("Show a supported tag. Press Ctrl+C to stop.")
    print("Audio plays from the Raspberry Pi running this script.")

    try:
        while True:
            frame = camera.capture_array()
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            detections = detector.detect(gray)
            now = time.monotonic()
            current_ids = {int(d.tag_id) for d in detections}

            for tag_id in current_ids:
                last_seen[tag_id] = now

            for tag_id in current_ids:
                audio_path = TAG_AUDIO.get(tag_id)
                if audio_path is None:
                    continue

                # Trigger on first appearance, then only after the tag has
                # disappeared long enough and the cooldown has elapsed.
                newly_seen = tag_id not in previous_ids
                cooled_down = (
                    now - last_triggered.get(tag_id, -1e9) >= COOLDOWN_SECONDS
                )

                if newly_seen and cooled_down:
                    print(f"Detected AprilTag {tag_id}; playing {audio_path.name}")
                    try:
                        # Do not interrupt a sound that is already playing.
                        if not pygame.mixer.music.get_busy():
                            pygame.mixer.music.load(str(audio_path))
                            pygame.mixer.music.play()
                            last_triggered[tag_id] = now
                        else:
                            print("Audio is already playing; skipping this trigger.")
                    except pygame.error as exc:
                        print(f"Could not play {audio_path}: {exc}")

            previous_ids = current_ids
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nStopping AprilTag audio add-on.")
    finally:
        camera.stop()
        pygame.mixer.quit()
    return 0

if __name__ == "__main__":
    sys.exit(main())
