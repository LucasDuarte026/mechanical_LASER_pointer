"""Lightweight benchmark utility for MediaPipe Hands.

The script mirrors the low-latency pathway used in ``controller.py`` so that
we can profile MediaPipe independently from the rest of the pipeline. It
overlays the 21 detected landmarks and reports both instantaneous FPS and model
latency.
"""

import time

import cv2
import mediapipe as mp


def main() -> None:
    """Run the MediaPipe Hands benchmark loop."""
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    mp_draw = mp.solutions.drawing_utils

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: could not open camera.")
        return

    print("Starting benchmark... Press 'q' to exit.")
    previous_time = 0.0

    try:
        while True:
            success, image = cap.read()
            if not success:
                print("Skipping empty frame from camera.")
                continue

            start_time = time.time()
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = hands.process(image_rgb)
            end_time = time.time()

            latency_ms = (end_time - start_time) * 1000
            current_time = time.time()
            fps = 1 / (current_time - previous_time) if previous_time else 0.0
            previous_time = current_time

            if results.multi_hand_landmarks:
                for hand_lms in results.multi_hand_landmarks:
                    mp_draw.draw_landmarks(image, hand_lms, mp_hands.HAND_CONNECTIONS)

            cv2.putText(
                image,
                f"FPS: {int(fps)}",
                (10, 70),
                cv2.FONT_HERSHEY_PLAIN,
                3,
                (255, 0, 0),
                3,
            )
            cv2.putText(
                image,
                f"Latency: {latency_ms:.2f} ms",
                (10, 110),
                cv2.FONT_HERSHEY_PLAIN,
                2,
                (0, 0, 255),
                2,
            )

            cv2.imshow("Benchmark MediaPipe", image)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        hands.close()


if __name__ == "__main__":
    main()
