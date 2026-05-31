"""YOLO 노드 공통 도움 함수.

규칙:
  1. 직접 만든 모델 경로가 없으면 미감지 상태 유지
  2. yolov8n.pt 같은 기본 모델 이름은 Ultralytics에 그대로 전달
  3. 여러 박스 중 가장 큰 박스 선택
"""

from pathlib import Path


def load_yolo_model(node, model_path):
    """YOLO 모델 로딩."""
    if not model_path:
        node.get_logger().warn('model_path 비어 있음')
        return None

    try:
        from ultralytics import YOLO
    except ImportError:
        node.get_logger().warn('ultralytics 패키지 없음')
        return None

    model_file = Path(model_path).expanduser()

    # 직접 만든 best.pt 같은 파일 경로 확인
    if model_file.is_absolute() or '/' in model_path:
        if not model_file.exists():
            node.get_logger().warn(f'YOLO 모델 파일 없음: {model_file}')
            return None
        model_source = str(model_file)
    else:
        # yolov8n.pt 같은 기본 모델 이름 사용
        model_source = model_path

    try:
        model = YOLO(model_source)
    except Exception as error:
        node.get_logger().warn(f'YOLO 모델 로딩 실패: {error}')
        return None

    node.get_logger().info(f'YOLO 모델 로딩 완료: {model_source}')
    return model


def resolve_yolo_device(node, requested_device):
    """YOLO 추론 장치 선택."""
    if requested_device != 'auto':
        node.get_logger().info(f'YOLO 추론 장치: {requested_device}')
        return requested_device

    try:
        import torch
    except ImportError:
        node.get_logger().info('YOLO 추론 장치: cpu')
        return 'cpu'

    if torch.cuda.is_available():
        device_name = torch.cuda.get_device_name(0)
        node.get_logger().info(f'YOLO 추론 장치: cuda:0 ({device_name})')
        return 0

    node.get_logger().info('YOLO 추론 장치: cpu')
    return 'cpu'


def biggest_box(result, target_class_name='rc_car'):
    """가장 큰 YOLO 박스 선택."""
    names = result.names
    best = None
    best_area = 0.0

    for box in result.boxes:
        class_id = int(box.cls[0])
        class_name = names.get(class_id, str(class_id))
        if class_name != target_class_name:
            continue

        x1, y1, x2, y2 = [float(value) for value in box.xyxy[0]]
        width = x2 - x1
        height = y2 - y1
        area = width * height
        if area <= best_area:
            continue

        track_id = -1
        if box.id is not None:
            track_id = int(box.id[0])

        best_area = area
        best = {
            'class_name': class_name,
            'confidence': float(box.conf[0]),
            'track_id': track_id,
            'center_x': x1 + width / 2.0,
            'center_y': y1 + height / 2.0,
            'width': width,
            'height': height,
        }

    return best
