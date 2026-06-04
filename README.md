# Mini1 RC Car Following Project

TurtleBot4가 고정 웹캠으로 RC카 등장을 감지한 뒤, 관찰 지점으로 이동하고, 로봇 OAK-D RGB-D 카메라로 RC카를 따라가는 ROS 2 Humble 프로젝트입니다.

## 핵심 흐름

```text
고정 USB 웹캠
  -> tripod_trigger_node
  -> /rc_car/webcam_detected
  -> approach_planner_node
  -> watch_area Nav2 goal
  -> watch_area 도착
  -> /rc_car/nav2_tracking_enabled true
  -> oakd_yolo_depth_node
  -> /rc_car/target/oakd_3d
  -> nav2_target_tracker_node
  -> /robot3/navigate_to_pose
  -> Nav2 기반 RC카 추적
```

현재 추적은 직접 `/cmd_vel`을 밀어 넣는 방식이 아니라 **Nav2 NavigateToPose goal을 갱신하는 방식**입니다. 그래서 Nav2 costmap과 장애물 회피를 그대로 사용합니다.

## 현재 동작 규칙

- 고정 웹캠은 `car`만 trigger로 사용합니다.
- 커스텀 YOLO 모델에 `dummy`, `charger` 클래스가 있어도 현재 노드는 `car`만 요청하므로 dummy/charger에는 반응하지 않습니다.
- 고정 웹캠이 `false -> true`로 바뀌는 순간 watch_area로 이동합니다.
- watch_area 도착 성공 시 OAK-D 기반 추적이 켜집니다.
- OAK-D가 `car`를 처음 확정하면 `/robot3/cmd_audio`로 삐뽀삐뽀가 한 번만 울립니다.
- 추적 중 OAK-D가 `car`를 연속 5프레임 못 보면 `/rc_car/return_to_watch_area`를 보내고 watch_area로 복귀합니다.
- 영상 디버그 토픽은 네트워크 보호를 위해 기본 OFF입니다.

## 패키지 구조

```text
mini1_ws/
  README.md
  src/
    rc_car_interfaces/
      msg/
        Target2D.msg
        Target3D.msg
        SystemState.msg
      srv/
        SetFollowState.srv

    rc_car_follower/
      rc_car_follower/
        tripod_trigger_node.py
        tripod_yolo_node.py
        oakd_yolo_depth_node.py
        approach_planner_node.py
        nav2_target_tracker_node.py
        follow_controller_node.py
        supervisor_node.py
        yolo_helper.py
        webcam_car_test.py

    rc_car_bringup/
      config/
        project.yaml
        watch_area_waypoint.yaml
        web_corners.yaml
      launch/
        project.launch.py
        project_nodes.launch.py
        web_control.launch.py
      maps/
        hoon_map.yaml
        hoon_map.pgm
      models/
        tripod_cam.pt   # git에는 포함되지 않음
        robot_cam.pt    # git에는 포함되지 않음
      web/
        index.html
        hoon_map.pgm
      docs/

    rc_car_web/
      rc_car_web/
        web_command_node.py
        web_pose_node.py
        web_map_node.py

    ros2_homework_examples/
```

## 모델 파일

`.pt` 모델은 `.gitignore`에 의해 GitHub에 올라가지 않습니다. 실행 전 로컬에 직접 배치해야 합니다.

```bash
mkdir -p ~/mini1_ws/src/rc_car_bringup/models
cp ~/Downloads/tripod_cam.pt ~/mini1_ws/src/rc_car_bringup/models/
cp ~/Downloads/robot_cam.pt ~/mini1_ws/src/rc_car_bringup/models/
```

현재 기준 클래스:

```text
tripod_cam.pt: car, dummy
robot_cam.pt: car, charger, dummy
```

프로젝트는 두 모델 모두 `target_class_name: car`만 사용합니다.

## 주요 설정

설정 파일:

```text
src/rc_car_bringup/config/project.yaml
```

중요한 값:

```yaml
tripod_trigger_node:
  ros__parameters:
    camera_device: auto
    device_id: 2
    model_path: /home/rokey/mini1_ws/src/rc_car_bringup/models/tripod_cam.pt
    target_class_name: car
    confidence_threshold: 0.45
    frame_rate: 5.0
    publish_debug_image: false
    publish_debug_compressed_image: false

oakd_yolo_depth_node:
  ros__parameters:
    model_path: /home/rokey/mini1_ws/src/rc_car_bringup/models/robot_cam.pt
    target_class_name: car
    confidence_threshold: 0.45
    inference_rate: 2.0
    publish_debug_image: false
    publish_debug_compressed_image: false

nav2_target_tracker_node:
  ros__parameters:
    return_when_lost: true
    return_missed_frames: 5
    beep_on_tracking: true
```

`camera_device: auto`는 `/dev/v4l/by-id/*`와 `/dev/video*` 중 열리는 카메라를 자동 선택합니다. 시연 전에는 로그의 `웹캠 자동 선택: ...` 줄을 확인하는 것이 좋습니다. 특정 웹캠으로 고정하려면 다음처럼 안정 경로를 직접 넣습니다.

```yaml
camera_device: /dev/v4l/by-id/usb-...-video-index0
```

watch_area 좌표:

```text
src/rc_car_bringup/config/watch_area_waypoint.yaml
```

웹 1~4번 이동 좌표:

```text
src/rc_car_bringup/config/web_corners.yaml
```

## 빌드

```bash
cd ~/mini1_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rc_car_interfaces rc_car_follower rc_car_bringup rc_car_web
source install/setup.bash
```

새 터미널마다:

```bash
cd ~/mini1_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 권장 실행 순서

강의실 네트워크가 불안정하므로 한 번에 다 켜는 `project.launch.py`보다 분리 실행을 권장합니다.

1. 지도/로컬라이제이션 실행

```bash
robot-hoon
```

2. RViz에서 `2D Pose Estimate`

3. Nav2 실행

```bash
robot-nav
```

4. Nav2 lifecycle 확인

```bash
ros2 lifecycle get /robot3/controller_server
ros2 lifecycle get /robot3/smoother_server
ros2 lifecycle get /robot3/planner_server
ros2 lifecycle get /robot3/behavior_server
ros2 lifecycle get /robot3/bt_navigator
ros2 lifecycle get /robot3/waypoint_follower
```

필요 시 수동 activate:

```bash
ros2 lifecycle set /robot3/smoother_server activate
ros2 lifecycle set /robot3/planner_server activate
ros2 lifecycle set /robot3/behavior_server activate
ros2 lifecycle set /robot3/bt_navigator activate
ros2 lifecycle set /robot3/waypoint_follower activate
```

5. 프로젝트 노드 실행

```bash
ros2 launch rc_car_bringup project_nodes.launch.py start_oakd:=true start_tracker:=true
```

`start_oakd`와 `start_tracker`는 기본값이 false이므로 추적까지 하려면 반드시 켭니다.

6. 웹 제어 실행

```bash
ros2 launch rc_car_bringup web_control.launch.py
```

7. 웹 페이지 서버 실행

```bash
cd ~/mini1_ws/install/rc_car_bringup/share/rc_car_bringup/web
python3 -m http.server 8000
```

브라우저:

```text
http://localhost:8000
```

## 가벼운 확인 명령

```bash
ros2 topic echo --once /rc_car/webcam_detected
ros2 topic echo --once /rc_car/target/oakd_3d
ros2 topic echo --once /rc_car/nav2_tracking_enabled
ros2 topic echo --once /rc_car/return_to_watch_area
```

카메라 장치 확인:

```bash
ls -l /dev/video*
ls -l /dev/v4l/by-id/
```

OAK-D confidence 확인:

```bash
ros2 topic echo --once /rc_car/target/oakd_3d
```

## rqt와 네트워크 주의

영상 디버그 토픽은 기본 OFF입니다. rqt나 `rqt_image_view`에서 영상을 보기 위해 디버그 이미지를 켜면 Wi-Fi 트래픽이 늘어납니다.

기본 권장:

```yaml
publish_debug_image: false
publish_debug_compressed_image: false
```

꼭 확인할 때만 압축 영상만 잠깐 켭니다.

```yaml
publish_debug_compressed_image: true
```

피하는 명령:

```bash
ros2 topic echo /robot3/oakd/rgb/preview/image_raw
ros2 topic echo /robot3/oakd/stereo/image_raw
rqt_image_view
```

시연 중에는 `robot-hoon`, RViz, `robot-nav`는 가능하면 끄지 말고, 문제가 생기면 `project_nodes.launch.py`와 `web_control.launch.py`만 먼저 재시작합니다.

## 주요 토픽과 액션

| 이름 | 종류 | 의미 |
|---|---|---|
| `/rc_car/webcam_detected` | Topic, `std_msgs/Bool` | 고정 웹캠 car 감지 여부 |
| `/rc_car/nav2_tracking_enabled` | Topic, `std_msgs/Bool` | OAK-D 추적 시작 여부 |
| `/rc_car/return_to_watch_area` | Topic, `std_msgs/Bool` | RC카 분실 시 watch_area 복귀 요청 |
| `/rc_car/target/oakd_3d` | Topic, `rc_car_interfaces/Target3D` | OAK-D가 본 car 거리/방향 |
| `/robot3/navigate_to_pose` | Action | Nav2 이동 명령 |
| `/robot3/cmd_audio` | Topic | 추적 시작 알림음 |
| `/robot3/dock` | Action | Dock 명령 |
| `/robot3/undock` | Action | Undock 명령 |
| `/rc_car/web/command` | Topic, `std_msgs/String` | 웹 버튼 명령 |
| `/rc_car/web/robot_pose` | Topic | 웹 표시용 로봇 위치 |

## 노드 역할

| 노드 | 역할 |
|---|---|
| `tripod_trigger_node` | 노트북 USB 웹캠에서 `car` 감지, `/rc_car/webcam_detected` 발행 |
| `approach_planner_node` | 고정 웹캠 trigger 또는 복귀 요청을 watch_area Nav2 goal로 변환 |
| `oakd_yolo_depth_node` | OAK-D RGB에서 `car` 감지, depth로 거리 계산 |
| `nav2_target_tracker_node` | OAK-D target을 Nav2 추적 goal로 변환, 분실 시 watch_area 복귀 요청 |
| `follow_controller_node` | 직접 `/cmd_vel` 추적 테스트용, 최종 주행 기본 방식 아님 |
| `web_command_node` | 웹 버튼을 Nav2/Dock/Undock 액션으로 변환 |
| `web_pose_node` | 로봇 위치를 웹으로 전달 |
| `web_map_node` | ROS map 재발행 보조 노드 |

## 문제 해결

### 웹캠 열기 실패

증상:

```text
Camera index out of range
웹캠 열기 실패: /dev/video2
```

확인:

```bash
ls -l /dev/video*
ls -l /dev/v4l/by-id/
```

해결:

- `camera_device: auto` 로그에서 선택된 장치를 확인합니다.
- 엉뚱한 카메라를 잡으면 `/dev/v4l/by-id/...` 경로를 `camera_device`에 직접 넣습니다.
- 카메라가 점유되어 있으면 `fuser -v /dev/videoN`으로 확인합니다.

### Undock 액션 서버 연결 실패

대개 웹 버튼 문제가 아니라 COMM 또는 Create3 액션 서버가 사라진 상태입니다.

```bash
ros2 action list -t | grep -E "dock|undock"
```

정상 예:

```text
/robot3/dock [irobot_create_msgs/action/Dock]
/robot3/undock [irobot_create_msgs/action/Undock]
```

직접 테스트:

```bash
ros2 action send_goal /robot3/undock irobot_create_msgs/action/Undock "{}"
```

### RViz Message Filter queue full

RViz가 표시를 따라가지 못하거나 네트워크가 밀릴 때 나옵니다. 카메라, PointCloud, 과한 costmap 표시를 끄고 Map, RobotModel, Path 정도만 남기는 것을 권장합니다.

### Nav2 active 안 됨

```bash
ros2 lifecycle get /robot3/bt_navigator
ros2 lifecycle get /robot3/controller_server
ros2 lifecycle get /robot3/planner_server
```

inactive면 lifecycle을 수동 activate합니다.

## 문서

추가 문서는 `src/rc_car_bringup/docs/`에 있습니다.

- `agent_handoff.md`: 코드 수정자용 인계 문서
- `web_control.md`: 웹 제어 구조
- `oakd_config_note.md`: OAK-D 설정 메모
- `architecture.md`: 시스템 구조
