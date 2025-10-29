"""High-level servo/vision controller for the laser hand follower prototype.

The module combines three concurrent concerns:
1. Camera processing (main thread) to locate the laser (YOLOv5) and the hand
   palm center (MediaPipe Hands).
2. A proportional controller that derives new servo targets from the spatial
   error between both detections.
3. A serial worker thread that streams throttled commands to an Arduino board.

Everything is purposefully kept in the same file to make it easier to tune the
prototype during development. The amount of logging is intentionally high so
that issues during workshops can be diagnosed without attaching a debugger.
"""

import sys
import threading
import time
import warnings
from pathlib import Path

import cv2
import mediapipe as mp
import torch

from SerialCommunicator import ArduinoSerialTerminal


warnings.filterwarnings("ignore", category=FutureWarning)

# Minimum pixel error required before the controller nudges a servo.
DEAD_ZONE_X = 2
DEAD_ZONE_Y = 2

# Frequency (Hz) at which the worker thread checks for outbound servo commands.
FREQUENCY_SERVO_CONTROL = 10.0

# Size of the discrete servo corrections performed whenever the error is larger
# than the defined dead zone. Values are normalized to the [0, 100] protocol.
SERVO_STEP = 1


def servos_control(terminal, shared_state, stop_event):
    """Background worker that streams setpoints to the Arduino.

    The worker runs at ``FREQUENCY_SERVO_CONTROL`` hertz and only forwards
    values that differ from the last command acknowledged by the Arduino. When a
    coordinate does not change we emit ``-1`` so the microcontroller knows it
    can reuse the previous value (see the Arduino sketch for details).
    """
    print("[Serial Thread] Started.")

    while not stop_event.wait(1.0 / FREQUENCY_SERVO_CONTROL):
        if not terminal or not terminal.running:
            continue

        with shared_state["lock"]:
            local_desired_x = shared_state["desired_x"]
            local_desired_y = shared_state["desired_y"]

            # Only send deltas to avoid saturating the serial line with
            # duplicated coordinates. The Arduino-side protocol interprets "-1"
            # as "hold".
            changed = False
            if local_desired_x != shared_state["last_sent_x"]:
                shared_state["last_sent_x"] = local_desired_x
                changed = True
            else:
                local_desired_x = -1

            if local_desired_y != shared_state["last_sent_y"]:
                shared_state["last_sent_y"] = local_desired_y
                changed = True
            else:
                local_desired_y = -1

        if changed:
            terminal.move_to(local_desired_x, local_desired_y)

    print("[Serial Thread] Stopped.")

def calculate_error(hand_x, hand_y, led_x, led_y):
    """Return the positional error between the hand and the LED crosshair.

    Args:
        hand_x: X coordinate of the estimated palm center (pixels).
        hand_y: Y coordinate of the estimated palm center (pixels).
        led_x: X coordinate of the LED detection (pixels).
        led_y: Y coordinate of the LED detection (pixels).

    Returns:
        Tuple``(error_x, error_y)`` representing the delta *LED - hand* in
        pixels. If either detection is missing the function returns ``(None,
        None)`` so the control loop can gracefully skip a frame.
    """
    if hand_x is None or hand_y is None or led_x is None or led_y is None:
        return None, None
    error_x = led_x - hand_x
    error_y = led_y - hand_y
    return error_x, error_y

def calculate_new_servo_targets(error_x, error_y, current_x, current_y):
    """Translate the error vector into normalized servo setpoints.

    The controller is intentionally conservative: we only take one normalized
    step per update (``SERVO_STEP``) and clamp to the [0, 100] range expected by
    the firmware. When either axis is missing we hold the previous value.
    """
    if error_x is None:
        return current_x, current_y

    new_target_x = current_x
    new_target_y = current_y

    if abs(error_x) > DEAD_ZONE_X:
        step_x = -SERVO_STEP if error_x > 0 else SERVO_STEP
        new_target_x = current_x + step_x

    if abs(error_y) > DEAD_ZONE_Y:
        step_y = SERVO_STEP if error_y > 0 else -SERVO_STEP
        new_target_y = current_y + step_y

    new_target_x = int(max(0, min(100, new_target_x)))
    new_target_y = int(max(0, min(100, new_target_y)))

    return new_target_x, new_target_y

def draw_crosshair(frame, x, y, color=(0, 255, 0), thickness=2):
    """Overlay a crosshair on the provided frame."""
    height, width = frame.shape[:2]
    cv2.line(frame, (0, int(y)), (width, int(y)), color, thickness)
    cv2.line(frame, (int(x), 0), (int(x), height), color, thickness)
    return frame


def main():
    """Entry point used when running the module as a script."""
    print("Loading YOLOv5 model...")
    weights_path = Path(__file__).resolve().parent / "yolo_models" / "yolov5s6_e400_b8_tvt302010_laser_v4.pt"
    try:
        model_yolo = torch.hub.load(
            "ultralytics/yolov5",
            "custom",
            path=str(weights_path),
        )
        print("YOLOv5 model loaded.")
    except Exception as exc:
        print(f"Failed to load YOLOv5 model: {exc}")
        sys.exit(1)

    print("Loading MediaPipe Hands model...")
    mp_hands = mp.solutions.hands
    model_hand = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    mp_draw = mp.solutions.drawing_utils
    print("MediaPipe model loaded.")

    # Configure primary camera feed.
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: could not open camera.")
        sys.exit(1)

    pTime = 0.0
    frame_count = 0

    print("\n=== Combined YOLO (LED) + MediaPipe (Hand) Detector ===")
    print("- Red crosshair = LED (YOLO)")
    print("- Blue crosshair = Hand palm (MediaPipe)")
    print("- Press 'q' to exit")
    print("Starting stream...")

    print("\n\t-> Opening serial communication with Arduino...")
    terminal = None
    port = ArduinoSerialTerminal.scan_for_arduino()
    if port:
        terminal = ArduinoSerialTerminal(baud_rate=9600)
        if terminal.start_terminal(port):
            print("Serial communication established with Arduino.")
        else:
            print("Failed to open the serial terminal.")
            print("Continue without servo control? (y/n): ", end="")
            choice = input().strip().lower()
            if choice != "y":
                return
    else:
        print("Arduino not found. Continue without servo control? (y/n): ", end="")
        choice = input().strip().lower()
        if choice != "y":
            return

    shared_state = {
        "desired_x": 50,
        "desired_y": 50,
        "last_sent_x": 50,
        "last_sent_y": 50,
        "lock": threading.Lock(),
    }

    stop_event = threading.Event()

    serial_thread = None
    if terminal:
        terminal.move_to(shared_state["desired_x"], shared_state["desired_y"])

        serial_thread = threading.Thread(
            target=servos_control,
            args=(terminal, shared_state, stop_event),
            daemon=True,
        )
        serial_thread.start()
        print("[Main] Serial command thread running.")

    last_known_led_x = None
    last_known_led_y = None
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: could not read frame.")
                break

            frame = cv2.flip(frame, 1)
            height, width = frame.shape[:2]

            start_yolo = time.time()
            results_yolo = model_yolo(frame)
            end_yolo = time.time()
            latency_yolo = (end_yolo - start_yolo) * 1000

            led_x, led_y = None, None
            predictions = results_yolo.pandas().xyxy[0]

            if len(predictions) > 0:
                best_detection = predictions.loc[predictions['confidence'].idxmax()]
                x1, y1, x2, y2 = best_detection['xmin'], best_detection['ymin'], best_detection['xmax'], best_detection['ymax']

                led_x = (x1 + x2) / 2
                led_y = (y1 + y2) / 2

                last_known_led_x = led_x
                last_known_led_y = led_y
            else:
                led_x = last_known_led_x
                led_y = last_known_led_y

            image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            start_mp = time.time()
            results_hand = model_hand.process(image_rgb)
            end_mp = time.time()
            latency_mp = (end_mp - start_mp) * 1000

            hand_x, hand_y = None, None
            if results_hand.multi_hand_landmarks:
                hand_lms = results_hand.multi_hand_landmarks[0]

                mp_draw.draw_landmarks(frame, hand_lms, mp_hands.HAND_CONNECTIONS)

                lm_wrist = hand_lms.landmark[mp_hands.HandLandmark.WRIST]
                lm_index = hand_lms.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP]
                lm_pinky = hand_lms.landmark[mp_hands.HandLandmark.PINKY_MCP]

                avg_x_normalized = (lm_wrist.x + lm_index.x + lm_pinky.x) / 3.0
                avg_y_normalized = (lm_wrist.y + lm_index.y + lm_pinky.y) / 3.0

                hand_x = avg_x_normalized * width
                hand_y = avg_y_normalized * height

            if led_x is not None:
                draw_crosshair(frame, led_x, led_y, color=(0, 0, 255), thickness=2)

            if hand_x is not None:
                draw_crosshair(frame, hand_x, hand_y, color=(255, 0, 0), thickness=2)

            cTime = time.time()
            fps = 1 / (cTime - pTime)
            pTime = cTime

            error_x, error_y = calculate_error(hand_x, hand_y, led_x, led_y)

            with shared_state["lock"]:
                current_x = shared_state["last_sent_x"]
                current_y = shared_state["last_sent_y"]

            new_target_x, new_target_y = calculate_new_servo_targets(error_x, error_y, current_x, current_y)

            with shared_state["lock"]:
                shared_state["desired_x"] = new_target_x
                shared_state["desired_y"] = new_target_y

            cv2.putText(
                frame,
                f"FPS: {int(fps)}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                frame,
                f"YOLO Latency: {latency_yolo:.2f} ms",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )
            cv2.putText(
                frame,
                f"MP Latency: {latency_mp:.2f} ms",
                (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )
            if error_x is not None:
                error_text = f"error_x: {error_x:.2f} | error_y: {error_y:.2f}"
            else:
                error_text = "error_x: None | error_y: None"
            cv2.putText(
                frame,
                error_text,
                (10, 500),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )

            cv2.imshow("YOLO + MediaPipe Detector", frame)

            frame_count += 1
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\nExiting...")
                break

    except KeyboardInterrupt:
        print("\nExiting... (Ctrl+C)")

    print("Starting cleanup...")
    stop_event.set()

    cap.release()
    cv2.destroyAllWindows()

    if serial_thread:
        print("Waiting for serial thread to stop...")
        serial_thread.join()

    if terminal:
        terminal.stop_terminal()

    if model_hand:
        model_hand.close()

    print("Program terminated.")

if __name__ == "__main__":
    main()
