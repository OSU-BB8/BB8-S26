
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
        # AUDIO: APRILTAG 0 ONLY
        # ======================================
        # Start audio once when Tag 0 first appears.
        # It can trigger again if Tag 0 disappears and reappears.
        tag_0_visible = len(tags) > 0

        if tag_0_visible and not previous_tag_0_visible and audio_ready:
            try:
                pygame.mixer.music.load(str(AUDIO_FILE))
                pygame.mixer.music.play()
                print("[AUDIO] Tag 0 detected - playing audio0.mp3")
            except Exception as exc:
                print(f"[AUDIO] Playback failed: {exc}")

        previous_tag_0_visible = tag_0_visible

        # ======================================
        # TERMINAL OUTPUT
        # ======================================

        now = time.monotonic()

        if now - last_print >= PRINT_INTERVAL:

            if results:

                print(
                    f"\nDetected {len(results)} target tag(s)"
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
                print("AprilTag 0 not detected")

            last_print = now

        # ======================================
        # VIDEO DISPLAY
        # ======================================

        if SHOW_VIDEO:

            cv2.putText(
                frame,
                f"Tag 0 detected: {'Yes' if tag_0_visible else 'No'}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2
            )

            cv2.imshow(
                "BB-8 AprilTag 0 Detection",
                frame
            )

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

except KeyboardInterrupt:
    print("\nStopping detector...")

finally:
    picam2.stop()
    cv2.destroyAllWindows()
