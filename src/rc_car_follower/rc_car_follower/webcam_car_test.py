"""USB 웹캠 YOLO car 감지 테스트 도구.

목적:
  1. USB 웹캠 연결 확인
  2. YOLO 기본 COCO 모델의 car 감지 확인
  3. RC카가 car로 잡히는지 사전 확인

실행:
  ros2 run rc_car_follower webcam_car_test

옵션 예시:
  ros2 run rc_car_follower webcam_car_test -- --device 2
  ros2 run rc_car_follower webcam_car_test -- --model yolov8n.pt
  ros2 run rc_car_follower webcam_car_test -- --no-window
"""

import argparse
import time

import cv2


COCO_CAR_CLASS_ID = 2


def parse_args():
    """터미널 옵션 읽기."""
    # argparse 객체 생성
    parser = argparse.ArgumentParser(description='Test USB webcam with YOLO COCO car detection.')

    # 웹캠 장치 번호
    parser.add_argument('--device', type=int, default=0, help='OpenCV camera index. Usually 0.')

    # YOLO 모델 이름 또는 경로
    parser.add_argument('--model', default='yolov8n.pt', help='YOLO model path or name.')

    # car 감지 최소 신뢰도
    parser.add_argument('--confidence', type=float, default=0.35, help='Minimum car confidence.')

    # 웹캠 요청 해상도
    parser.add_argument('--width', type=int, default=640, help='Requested camera width.')
    parser.add_argument('--height', type=int, default=480, help='Requested camera height.')

    # 화면 표시 끄기 옵션
    parser.add_argument('--no-window', action='store_true', help='Print results without showing a window.')
    return parser.parse_args()


def load_model(model_name):
    """YOLO 모델 로딩."""
    try:
        # ultralytics YOLO import
        from ultralytics import YOLO
    except ImportError as error:
        raise RuntimeError('ultralytics is not installed. Install it before YOLO testing.') from error

    # YOLO 모델 객체 생성
    return YOLO(model_name)


def draw_car_boxes(frame, result):
    """car 박스 화면 표시."""
    # 감지된 car 개수
    car_count = 0

    for box in result.boxes:
        # YOLO class id 확인
        class_id = int(box.cls[0])
        if class_id != COCO_CAR_CLASS_ID:
            continue

        # 박스 좌표와 신뢰도 읽기
        confidence = float(box.conf[0])
        x1, y1, x2, y2 = [int(value) for value in box.xyxy[0]]
        label = f'car {confidence:.2f}'
        car_count += 1

        # 박스 그리기
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 220, 0), 2)

        # 라벨 그리기
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
    # 터미널 옵션 읽기
    args = parse_args()

    print('[webcam_car_test] Loading YOLO model:', args.model)
    print('[webcam_car_test] First run may download yolov8n.pt if it is not cached.')

    # YOLO 모델 준비
    model = load_model(args.model)

    # USB 웹캠 열기
    camera = cv2.VideoCapture(args.device)

    # 웹캠 해상도 요청
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    # 웹캠 열기 실패 처리
    if not camera.isOpened():
        print(f'[webcam_car_test] Cannot open /dev/video{args.device}.')
        print('[webcam_car_test] Check USB connection with: ls -l /dev/video*')
        return

    print('[webcam_car_test] Camera opened.')
    print('[webcam_car_test] Press q in the window to quit.')

    last_print_time = 0.0
    while True:
        # 웹캠 프레임 읽기
        ok, frame = camera.read()
        if not ok:
            print('[webcam_car_test] Failed to read a frame.')
            break

        # YOLO car 감지
        results = model.predict(
            frame,
            classes=[COCO_CAR_CLASS_ID],
            conf=args.confidence,
            verbose=False,
        )

        # 감지 박스 표시
        car_count = draw_car_boxes(frame, results[0])

        # 1초 간격 감지 개수 출력
        now = time.monotonic()
        if now - last_print_time > 1.0:
            print(f'[webcam_car_test] detected car count: {car_count}')
            last_print_time = now

        # 디버그 화면 표시
        if not args.no_window:
            cv2.imshow('webcam_car_test - COCO car detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # 웹캠 자원 정리
    camera.release()

    # OpenCV 창 정리
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
