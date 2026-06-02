# OAK-D 설정 메모

## 이 문서의 목적

로봇 라즈베리파이에 있던 OAK-D 설정 YAML의 의미 정리.

이 설정은 OAK-D 카메라가 어떤 화면을 만들고, Depth가 RGB에 맞춰지는지 확인할 때 사용.

## 원본 설정

```yaml
/oakd:
  ros__parameters:
    camera:
      i_enable_imu: false
      i_enable_ir: false
      i_floodlight_brightness: 0
      i_laser_dot_brightness: 100
      i_nn_type: none
      i_pipeline_type: RGBD
      i_usb_speed: SUPER_PLUS
    rgb:
      i_board_socket_id: 0
      i_fps: 10.0
      i_height: 704
      i_interleaved: false
      i_max_q_size: 10
      i_preview_size: 320
      i_enable_preview: true
      i_low_bandwidth: true
      i_keep_preview_aspect_ratio: true
      i_publish_topic: true
      i_resolution: '640P'
      i_width: 704
    use_sim_time: false
    stereo:
      i_publish_topic: true
      i_align_depth: true
      i_fps: 10.0
```

## 중요한 설정 의미

### camera.i_pipeline_type: RGBD

RGB 화면과 Depth 화면을 같이 사용하는 설정.

```text
RGB  → 색깔 카메라 화면
Depth → 거리 화면
```

우리 프로젝트에서는 RGB 화면에서 YOLO로 RC카를 찾고, Depth 화면에서 RC카까지의 거리를 읽음.

### camera.i_nn_type: none

OAK-D 내부 AI 모델을 사용하지 않는 설정.

우리 프로젝트에서는 OAK-D 내부 NN이 아니라 노트북 Python 코드에서 YOLO를 실행함.

### rgb.i_fps: 10.0

RGB 화면 초당 10장 발행.

초당 화면 수가 너무 높으면 와이파이와 CPU가 무거워질 수 있음.

### rgb.i_width / rgb.i_height: 704

RGB 카메라가 내부에서 사용하는 이미지 크기 설정.

```text
가로 704 픽셀
세로 704 픽셀
```

단, 실제 ROS 토픽 화면 크기는 토픽의 `camera_info` 또는 `image_raw` 메시지로 다시 확인 필요.

### rgb.i_preview_size: 320

미리보기 화면 크기 설정.

우리 코드의 RGB 입력 토픽은 보통 아래 토픽 사용.

```text
/robot3/oakd/rgb/preview/image_raw
```

따라서 YOLO가 보는 화면은 큰 원본 화면이 아니라 preview 화면일 수 있음.

### rgb.i_low_bandwidth: true

낮은 네트워크 사용량 모드.

와이파이 환경에서 영상 전송 렉을 줄이기 위한 설정.

### rgb.i_enable_preview: true

preview 화면 발행 활성화.

이 설정 덕분에 아래 토픽을 사용할 수 있음.

```text
/robot3/oakd/rgb/preview/image_raw
```

### stereo.i_publish_topic: true

Depth 토픽 발행 활성화.

이 설정 덕분에 아래 토픽을 사용할 수 있음.

```text
/robot3/oakd/stereo/image_raw
```

### stereo.i_align_depth: true

Depth 화면을 RGB 화면 기준에 맞추는 설정.

쉽게 말하면:

```text
RGB 화면의 자동차 위치
Depth 화면의 거리 위치
```

이 둘이 최대한 같은 위치를 가리키도록 맞추는 설정.

우리 프로젝트에서 매우 중요함.

## RGB와 Depth 크기 확인 이유

YOLO는 RGB 화면에서 RC카 박스 중심을 찾음.

```text
예: RGB 화면에서 RC카 중심 x=160, y=120
```

그다음 Depth 화면에서 같은 위치의 거리값을 읽음.

```text
예: Depth 화면 x=160, y=120 거리 = 0.72m
```

그런데 RGB 화면 크기와 Depth 화면 크기가 다르면 같은 좌표를 그대로 쓰면 안 됨.

예시:

```text
RGB 화면:   320 x 320
Depth 화면: 640 x 400
```

이 경우 RGB 좌표를 Depth 화면 크기에 맞게 비율 변환해야 함.

## 우리 코드의 처리 방식

우리 코드는 RGB와 Depth 크기가 달라도 동작하도록 비율 변환을 적용함.

파일:

```text
rc_car_follower/oakd_yolo_depth_node.py
```

핵심 코드:

```python
scale_x = depth_width / max(float(rgb_width), 1.0)
scale_y = depth_height / max(float(rgb_height), 1.0)
depth_x = pixel_x * scale_x
depth_y = pixel_y * scale_y
```

의미:

```text
RGB 좌표를 Depth 좌표로 변환
```

## 확인 명령어

RGB preview 크기 확인:

```bash
ros2 topic echo --once /robot3/oakd/rgb/preview/image_raw --field width
ros2 topic echo --once /robot3/oakd/rgb/preview/image_raw --field height
```

Depth 크기 확인:

```bash
ros2 topic echo --once /robot3/oakd/stereo/image_raw --field width
ros2 topic echo --once /robot3/oakd/stereo/image_raw --field height
```

## 발표용 한 줄 설명

OAK-D 설정에서 RGBD 파이프라인과 align_depth가 켜져 있어서 RGB 화면과 Depth 거리 화면을 같이 사용할 수 있고, 우리 코드는 두 화면 크기가 달라도 좌표 비율을 맞춰서 RC카 거리값을 읽도록 만들었다.
