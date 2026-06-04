"""USB 웹캠 YOLO car 감지 테스트 도구.

목적:
  1. USB 웹캠 연결 확인
  2. YOLO 모델의 car 감지 확인
  3. RC카만 car로 잡히는지 사전 확인

실행:
  ros2 run rc_car_follower webcam_car_test

옵션 예시:
  ros2 run rc_car_follower webcam_car_test -- --device 2
  ros2 run rc_car_follower webcam_car_test -- --camera-device CAMERA_PATH
  ros2 run rc_car_follower webcam_car_test -- --model yolov8n.pt
  ros2 run rc_car_follower webcam_car_test -- --no-window
"""

import argparse
import time

import cv2


def parse_args():
    """터미널 옵션 읽기."""
    # argparse 객체 생성
    parser = argparse.ArgumentParser(
        description='Test USB webcam with YOLO car detection.',
    )

    # 웹캠 장치 번호
    parser.add_argument(
        '--device',
        type=int,
        default=0,
        help='OpenCV camera index. Usually 0.',
    )

    # USB 재연결에도 유지되는 웹캠 장치 경로
    parser.add_argument(
        '--camera-device',
        default='',
        help='Stable camera path such as /dev/v4l/by-id/...',
    )

    # YOLO 모델 이름 또는 경로
    parser.add_argument(
        '--model',
        default='yolov8n.pt',
        help='YOLO model path or name.',
    )

    # 찾을 YOLO 클래스 이름
    parser.add_argument(
        '--target-class',
        default='car',
        help='YOLO class name to detect.',
    )

    # car 감지 최소 신뢰도
    parser.add_argument(
        '--confidence',
        type=float,
        default=0.35,
        help='Minimum car confidence.',
    )

    # 웹캠 요청 해상도
    parser.add_argument(
        '--width',
        type=int,
        default=640,
        help='Requested camera width.',
    )
    parser.add_argument(
        '--height',
        type=int,
        default=480,
        help='Requested camera height.',
    )

    # 화면 표시 끄기 옵션
    parser.add_argument(
        '--no-window',
        action='store_true',
        help='Print results without showing a window.',
    )
    return parser.parse_args()


def load_model(model_name):
    """YOLO 모델 로딩."""
    try:
        # ultralytics YOLO import
        from ultralytics import YOLO
    except ImportError as error:
        raise RuntimeError(
            'ultralytics is not installed. Install it before YOLO testing.'
        ) from error

    # YOLO 모델 객체 생성
    return YOLO(model_name)


def class_ids_for_name(model, target_class_name):
    """YOLO 모델에서 target class id 찾기."""
    names = getattr(model, 'names', {})
    if isinstance(names, list):
        names = dict(enumerate(names))

    class_ids = [
        int(class_id)
        for class_id, class_name in names.items()
        if class_name == target_class_name
    ]

    if class_ids:
        print(f'[webcam_car_test] target class ids: {class_ids}')
        return class_ids

    print(f'[webcam_car_test] target class not found: {target_class_name}')
    print(f'[webcam_car_test] model classes: {names}')
    return None


def draw_target_boxes(frame, result, target_class_name):
    """car 박스 화면 표시."""
    # 감지된 car 개수
    car_count = 0
    names = result.names
    if isinstance(names, list):
        names = dict(enumerate(names))

    for box in result.boxes:
        # YOLO class 이름 확인
        class_id = int(box.cls[0])
        class_name = names.get(class_id, str(class_id))
        if class_name != target_class_name:
            continue

        # 박스 좌표와 신뢰도 읽기
        confidence = float(box.conf[0])
        x1, y1, x2, y2 = [int(value) for value in box.xyxy[0]]
        label = f'{class_name} {confidence:.2f}'
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
    print(
        '[webcam_car_test] First run may download yolov8n.pt '
        'if it is not cached.'
    )

    # YOLO 모델 준비
    model = load_model(args.model)
    target_class_ids = class_ids_for_name(model, args.target_class)

    # USB 웹캠 열기
    camera_source = args.camera_device.strip() or args.device
    camera_source_text = (
        args.camera_device.strip()
        or f'/dev/video{args.device}'
    )
    camera = cv2.VideoCapture(camera_source)

    # 웹캠 해상도 요청
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    # 웹캠 열기 실패 처리
    if not camera.isOpened():
        print(f'[webcam_car_test] Cannot open {camera_source_text}.')
        print('[webcam_car_test] Check USB: ls -l /dev/video*')
        print('[webcam_car_test] Check stable names: ls -l /dev/v4l/by-id/')
        return

    print(f'[webcam_car_test] Camera opened: {camera_source_text}')
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
            classes=target_class_ids,
            conf=args.confidence,
            verbose=False,
        )

        # 감지 박스 표시
        car_count = draw_target_boxes(frame, results[0], args.target_class)

        # 1초 간격 감지 개수 출력
        now = time.monotonic()
        if now - last_print_time > 1.0:
            print(f'[webcam_car_test] detected car count: {car_count}')
            last_print_time = now

        # 디버그 화면 표시
        if not args.no_window:
            cv2.imshow('webcam_car_test - YOLO car detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # 웹캠 자원 정리
    camera.release()

    # OpenCV 창 정리
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
