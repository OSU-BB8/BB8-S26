import time
import cv2
from picamera2 import Picamera2
from pupil_apriltags import Detector
from picarx import Picarx

import serial 
import json 
import threading 

# ============================================================
# SETTINGS
# ============================================================

TAG_ID = 0                 # The ONE AprilTag to follow
DRIVE_SPEED = 25           # Start slow: PiCar-X motor power
TARGET_DISTANCE_CM = 60    # Try to stay about this far away

# Steering
MAX_STEERING = 30          # PiCar-X steering limit to use
CENTER_TOLERANCE = 50      # Pixels left/right from center

# Ultrasonic safety
STOP_DISTANCE_CM = 20      # Emergency stop distance

# Camera
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

UWB_PORT = "/dev/ttyACM0"  
UWB_BAUD = 115200
UWB_X_TOLERENCE_CM = 15
UWB_TIMEOUT_SECONDS = 1.0
# ============================================================
# START PICAR-X
# ============================================================

px = Picarx()

# Make sure robot starts stopped and straight
px.stop()
px.set_dir_servo_angle(0)

uwb_lock = threading.Lock()
uwb_dist_cm = None
uwb_x_cm = None
uwb_last_update = 0

def uwb_reader_thread():
    global uwb_dist_cm, uwb_x_cm, uwb_last_update
    try:
        ser = serial.Serial(UWB_PORT, UWB_BAUD, timeout=0.1)
    except Exception as e:
        print(f"UWB SERIAL FAILED TO OPEN: {e}")
        return
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if "{" in line:
                json_str = line[line.index("{"):]
                data = json.loads(json_str)["TWR"]
                with uwb_lock:
                    uwb_dist_cm = data["D"]
                    uwb_x_cm = data["Xcm"]
                    uwb_last_update = time.time()
        except (ValueError, KeyError, UnicodeDecodeError):
            continue
threading.Thread(target=uwb_reader_thread, daemon=True).start()
# ============================================================
# START CAMERA
# ============================================================

picam2 = Picamera2()

config = picam2.create_preview_configuration(
    main={"size": (FRAME_WIDTH, FRAME_HEIGHT)}
)

picam2.configure(config)
picam2.start()

time.sleep(2)


# ============================================================
# START APRILTAG DETECTOR
# ============================================================

detector = Detector(
    families="tag36h11"
)


print("========================================")
print(" APRILTAG FOLLOWER READY")
print(" Following AprilTag ID:", TAG_ID)
print(" Press CTRL+C to stop")
print("========================================")


# ============================================================
# MAIN LOOP
# ============================================================

try:

    while True:

        # ----------------------------------------------------
        # 1. READ ULTRASONIC SENSOR
        # ----------------------------------------------------

        try:
            obstacle_distance = px.ultrasonic.read()
        except Exception:
            obstacle_distance = 999


        # ----------------------------------------------------
        # 2. CAMERA FRAME
        # ----------------------------------------------------

        frame = picam2.capture_array()

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_RGB2GRAY
        )

        display = cv2.cvtColor(
            frame,
            cv2.COLOR_RGB2BGR
        )
        
        with uwb_lock:
            uwb_age = time.time() - uwb_last_update
            dist_cm = uwb_dist_cm
            x_cm = uwb_x_cm
            uwb_fresh = (uwb_age < UWB_TIMEOUT_SECONDS and dist_cm is not None)
        


        # ----------------------------------------------------
        # 3. FIND APRILTAGS
        # ----------------------------------------------------

        tags = detector.detect(gray)

        target = None

        for tag in tags:

            # Only follow the chosen tag
            if tag.tag_id == TAG_ID:
                target = tag
                break


        # ----------------------------------------------------
        # 4. SAFETY: OBSTACLE TOO CLOSE
        # ----------------------------------------------------

        if (
            obstacle_distance > 0
            and obstacle_distance < STOP_DISTANCE_CM
        ):

            px.stop()
            px.set_dir_servo_angle(0)

            cv2.putText(
                display,
                "OBSTACLE - STOP",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2
           )
           
        elif uwb_fresh:
                
                if abs(x_cm) < UWB_X_TOLERENCE_CM:
                    steering = 0
                else:
                    steering = int((x_cm/ 100) * MAX_STEERING)
                    steering = max(-MAX_STEERING, min(MAX_STEERING, steering))
                    
                px.set_dir_servo_angle(steering)
                
                if dist_cm > (TARGET_DISTANCE_CM + 15):
                    px.forward(DRIVE_SPEED)
                    state = "UWB FOLLOWING" 
                elif dist_cm < (TARGET_DISTANCE_CM - 15):
                     px.forward(-DRIVE_SPEED)
                     state = "UWB BACKING UP"
                    
                else:
                    px.stop()
                    state = "UWB TARGET REACHED"
                
                cv2.putText(
                    display, 
                    f"UWB: {state} dist={dist_cm}cm x={x_cm}cm steer:{steering}",
                    (20,40),
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.7,
                    (255, 255, 0),
                    2
                    )
                    
        # ----------------------------------------------------
        # 5. NO TAG FOUND
        # ----------------------------------------------------

        elif target is None:

            px.stop()
            px.set_dir_servo_angle(0)

            cv2.putText(
                display,
                "NO UWB/TAG LOST - STOP",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2
            )


        # ----------------------------------------------------
        # 6. APRILTAG FOUND
        # ----------------------------------------------------

        else:

            # Tag position
            tag_x = target.center[0]
            tag_y = target.center[1]

            # Camera center
            frame_center_x = FRAME_WIDTH / 2

            # How far tag is from center
            error_x = tag_x - frame_center_x
           
            # ------------------------------------------------
            # STEERING
            # ------------------------------------------------

            if abs(error_x) < CENTER_TOLERANCE:

                # Tag is approximately centered
                steering = 0

            else:

                # Convert position error into steering
                steering = int(
                    (error_x / frame_center_x)
                    * MAX_STEERING
                )

                # Limit steering
                steering = max(
                    -MAX_STEERING,
                    min(MAX_STEERING, steering)
                )


            px.set_dir_servo_angle(steering)


            # ------------------------------------------------
            # ESTIMATE TAG SIZE
            # ------------------------------------------------

            # Use width of detected tag as a simple
            # distance approximation.
            #
            # Bigger tag = closer
            # Smaller tag = farther

            corners = target.corners

            tag_width_pixels = abs(
                corners[1][0] - corners[0][0]
            )


            # ------------------------------------------------
            # FOLLOWING LOGIC
            # ------------------------------------------------

            # These numbers will probably need tuning
            # for your printed tag size.

            if tag_width_pixels < 100:

                # Tag is far away
                px.forward(DRIVE_SPEED)
                state = "FOLLOWING"

            elif tag_width_pixels < 180:

                # Getting close - slow down
                px.forward(15)
                state = "SLOW FOLLOW"

            else:

                # Close enough
                px.stop()
                state = "TARGET REACHED"


            # ------------------------------------------------
            # DRAW INFORMATION
            # ------------------------------------------------

            cv2.circle(
                display,
                (int(tag_x), int(tag_y)),
                10,
                (0, 255, 0),
                -1
            )

            cv2.putText(
                display,
                f"Tag {TAG_ID}",
                (int(tag_x) - 50, int(tag_y) - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            cv2.putText(
                display,
                f"{state}  steer:{steering}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )


        # ----------------------------------------------------
        # 7. DISPLAY CAMERA
        # ----------------------------------------------------

        cv2.imshow(
            "PiCar-X AprilTag Follow",
            display
        )

        # ESC stops the program
        if cv2.waitKey(1) == 27:
            break


except KeyboardInterrupt:

    print("\nStopping robot...")


finally:

    # IMPORTANT: always stop robot

    px.stop()
    px.set_dir_servo_angle(0)

    picam2.stop()

    cv2.destroyAllWindows()

    print("Robot stopped safely.")
