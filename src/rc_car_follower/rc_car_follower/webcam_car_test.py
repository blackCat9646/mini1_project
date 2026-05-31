"""Quick USB webcam + YOLO car detection test.

This is not the final robot behavior node.
It is a small test tool for answering one question:

  "Does the default COCO YOLO model detect our RC car as car?"

Run:
  ros2 run rc_car_follower webcam_car_test

Useful options:
  ros2 run rc_car_follower webcam_car_test -- --device 0
  ros2 run rc_car_follower webcam_car_test -- --model yolov8n.pt
  ros2 run rc_car_follower webcam_car_test -- --no-window
"""

import argparse
import time

import cv2


COCO_CAR_CLASS_ID = 2


def parse_args():
    parser = argparse.ArgumentParser(description='Test USB webcam with YOLO COCO car detection.')
    parser.add_argument('--device', type=int, default=0, help='OpenCV camera index. Usually 0.')
    parser.add_argument('--model', default='yolov8n.pt', help='YOLO model path or name.')
    parser.add_argument('--confidence', type=float, default=0.35, help='Minimum car confidence.')
    parser.add_argument('--width', type=int, default=640, help='Requested camera width.')
    parser.add_argument('--height', type=int, default=480, help='Requested camera height.')
    parser.add_argument('--no-window', action='store_true', help='Print results without showing a window.')
    return parser.parse_args()


def load_model(model_name):
    """Load YOLO only inside the test, so normal ROS nodes can still build without it."""
    try:
        from ultralytics import YOLO
    except ImportError as error:
        raise RuntimeError('ultralytics is not installed. Install it before YOLO testing.') from error

    return YOLO(model_name)


def draw_car_boxes(frame, result):
    """Draw only COCO class 2, which is car."""
    car_count = 0

    for box in result.boxes:
        class_id = int(box.cls[0])
        if class_id != COCO_CAR_CLASS_ID:
            continue

        confidence = float(box.conf[0])
        x1, y1, x2, y2 = [int(value) for value in box.xyxy[0]]
        label = f'car {confidence:.2f}'
        car_count += 1

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 220, 0), 2)
        cv2.putText(
            frame,
            label,
            (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 220, 0),
            2,
        )

    return car_count


def main():
    args = parse_args()

    print('[webcam_car_test] Loading YOLO model:', args.model)
    print('[webcam_car_test] First run may download yolov8n.pt if it is not cached.')
    model = load_model(args.model)

    camera = cv2.VideoCapture(args.device)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    if not camera.isOpened():
        print(f'[webcam_car_test] Cannot open /dev/video{args.device}.')
        print('[webcam_car_test] Check USB connection with: ls -l /dev/video*')
        return

    print('[webcam_car_test] Camera opened.')
    print('[webcam_car_test] Press q in the window to quit.')

    last_print_time = 0.0
    while True:
        ok, frame = camera.read()
        if not ok:
            print('[webcam_car_test] Failed to read a frame.')
            break

        results = model.predict(
            frame,
            classes=[COCO_CAR_CLASS_ID],
            conf=args.confidence,
            verbose=False,
        )
        car_count = draw_car_boxes(frame, results[0])

        now = time.monotonic()
        if now - last_print_time > 1.0:
            print(f'[webcam_car_test] detected car count: {car_count}')
            last_print_time = now

        if not args.no_window:
            cv2.imshow('webcam_car_test - COCO car detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    camera.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
