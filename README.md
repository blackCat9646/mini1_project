# Mini1 Project

TurtleBot4가 RC카를 찾고, 가까이 가고, 따라가는 미니 프로젝트.

이 문서는 처음 보는 사람도 실행 순서와 구조를 이해할 수 있도록 쉽게 정리한 문서.

## 한 줄 설명

고정 USB 웹캠이 RC카를 발견하면 TurtleBot4가 미리 찍어 둔 관찰 지점으로 이동하고, 그 뒤 로봇에 달린 OAK-D Depth 카메라로 RC카를 계속 보면서 Nav2로 따라가는 시스템.

## 현재 되는 기능

- 고정 USB 웹캠 YOLO 감지
- RC카 감지 신호 발행
- 감지 후 `watch_area` 지점으로 Nav2 이동
- OAK-D RGB + Depth 기반 RC카 거리 계산
- OAK-D 감지 결과 토픽 발행
- Nav2 기반 RC카 추적
- 웹 지도 표시
- 웹 지도 위 로봇 위치 표시
- 웹 지도 위 1, 2, 3, 4번 호출 위치 표시
- 웹 버튼 1, 2, 3, 4번 위치 이동
- 웹 Dock / Undock 버튼
- YOLO 디버그 이미지 토픽 발행
- ROS2 숙제용 예제 패키지 포함

## 중요한 동작 규칙

차가 감지되지 않으면 로봇은 현재 위치에서 대기.

차가 고정웹캠에 연속으로 감지되면 로봇은 `watch_area_waypoint.yaml`에 저장된 지점으로 이동.

로봇이 `watch_area`에 도착하면 OAK-D 기반 추적 모드 시작.

OAK-D가 RC카를 보면 Nav2 목표를 계속 갱신하면서 따라감.

## 실행 전 약속

한 로봇은 한 명만 제어.

다른 사람이 같은 `/robot3`에 대해 `nav2`, `localization`, `teleop`, `dock`, `undock`, `navigate_to_pose`를 실행하면 충돌 가능성 있음.

토픽 확인은 대부분 괜찮지만, OAK-D raw 이미지를 여러 명이 동시에 보면 와이파이와 로봇 Raspberry Pi에 부담 가능성 있음.

## 사용 환경

- Ubuntu 22.04
- ROS 2 Humble
- TurtleBot4
- namespace: `/robot3`
- map: `hoon_map`
- 고정 USB 웹캠: 노트북 연결
- 로봇 카메라: OAK-D
- 로봇 LiDAR: RPLiDAR
- YOLO 모델: `yolov8n.pt`

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
        approach_planner_node.py
        oakd_yolo_depth_node.py
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
      web/
        index.html
        hoon_map.pgm
      docs/
        web_control.md
        oakd_config_note.md

    rc_car_web/
      rc_car_web/
        web_command_node.py
        web_pose_node.py
        web_map_node.py

    ros2_homework_examples/
      # ROS2 수업 숙제용 임시 패키지
      # 검사 후 삭제 가능
```

## 주요 설정 파일

### 고정 웹캠 및 YOLO 설정

파일:

```text
src/rc_car_bringup/config/project.yaml
```

중요 값:

```yaml
tripod_trigger_node:
  ros__parameters:
    device_id: 2
    model_path: /home/rokey/mini1_ws/yolov8n.pt
    confidence_threshold: 0.35
    required_consecutive_detections: 5
    allowed_consecutive_misses: 10
```

의미:

```text
device_id: 2
→ /dev/video2 사용

required_consecutive_detections: 5
→ RC카가 5번 연속 감지되어야 true

allowed_consecutive_misses: 10
→ 잠깐 안 보여도 바로 false로 바꾸지 않음
```

### 관찰 지점 설정

파일:

```text
src/rc_car_bringup/config/watch_area_waypoint.yaml
```

현재 값:

```yaml
watch_area:
  frame_id: map
  x: -1.628544569015503
  y: 1.994612455368042
  yaw: 0.0
```

의미:

```text
고정웹캠이 RC카를 발견했을 때 로봇이 먼저 이동할 위치
```

### 웹 1, 2, 3, 4번 위치 설정

파일:

```text
src/rc_car_bringup/config/web_corners.yaml
```

의미:

```text
웹에서 1, 2, 3, 4 버튼을 눌렀을 때 이동할 위치
```

## 시스템 아키텍처

```text
고정 USB 웹캠
  ↓ 이미지
tripod_trigger_node
  ↓ /rc_car/webcam_detected
approach_planner_node
  ↓ /robot3/navigate_to_pose Action
Nav2
  ↓ /robot3/cmd_vel
TurtleBot4 이동
  ↓ watch_area 도착
nav2_tracking_enabled = true
  ↓
OAK-D RGB + Depth
  ↓ 이미지 + 깊이값
oakd_yolo_depth_node
  ↓ /rc_car/target/oakd_3d
nav2_target_tracker_node
  ↓ /robot3/navigate_to_pose Action
Nav2 기반 RC카 추적
```

웹 구조:

```text
웹 브라우저
  ↓ 버튼 클릭
rosbridge_server
  ↓ /rc_car/web/command
web_command_node
  ↓ NavigateToPose / Dock / Undock Action
TurtleBot4

웹 브라우저
  ↓ 정적 파일 읽기
hoon_map.pgm
  ↓
웹 지도 표시

web_pose_node
  ↓ /rc_car/web/robot_pose
웹 로봇 위치 표시
```

## 노드 역할

| 노드 | 역할 | 입력 | 출력 |
|---|---|---|---|
| `tripod_trigger_node` | 고정웹캠으로 RC카 발견 여부 판단 | USB 웹캠 | `/rc_car/webcam_detected` |
| `approach_planner_node` | 감지 신호를 watch_area 이동 명령으로 변환 | `/rc_car/webcam_detected` | `/robot3/navigate_to_pose` |
| `oakd_yolo_depth_node` | OAK-D RGB에서 RC카 찾기, Depth로 거리 계산 | OAK-D RGB/Depth | `/rc_car/target/oakd_3d` |
| `nav2_target_tracker_node` | OAK-D 감지값을 Nav2 추적 목표로 변환 | `/rc_car/target/oakd_3d` | `/robot3/navigate_to_pose` |
| `web_command_node` | 웹 버튼 명령 처리 | `/rc_car/web/command` | Nav2/Dock/Undock Action |
| `web_pose_node` | 로봇 위치를 웹으로 전달 | TF 또는 AMCL pose | `/rc_car/web/robot_pose` |
| `web_map_node` | ROS 맵 재발행용 보조 노드 | `/robot3/map` | `/rc_car/web/map` |

참고:

현재 웹 배경 지도는 안정성을 위해 ROS map_server를 기다리지 않고 `web/hoon_map.pgm` 파일을 직접 표시.

## 주요 토픽과 액션

| 이름 | 종류 | 의미 |
|---|---|---|
| `/rc_car/webcam_detected` | Topic, `Bool` | 고정웹캠 RC카 감지 여부 |
| `/rc_car/target/oakd_3d` | Topic, `Target3D` | OAK-D가 본 RC카 거리와 방향 |
| `/rc_car/nav2_tracking_enabled` | Topic, `Bool` | OAK-D 추적 시작 여부 |
| `/robot3/navigate_to_pose` | Action | Nav2 이동 명령 |
| `/robot3/cmd_vel` | Topic | 실제 로봇 속도 명령 |
| `/robot3/dock` | Action | Dock 명령 |
| `/robot3/undock` | Action | Undock 명령 |
| `/rc_car/web/command` | Topic, `String` | 웹 버튼 명령 |
| `/rc_car/web/robot_pose` | Topic, `PoseStamped` | 웹 표시용 로봇 위치 |

## 전체 플로우차트

```text
[시작]
  ↓
[robot-hoon 실행]
  ↓
{맵 로드 성공?}
  ├─ No → map_server 확인 → 다시 실행
  └─ Yes
       ↓
[RViz 2D Pose Estimate]
       ↓
{AMCL 위치 추정 성공?}
  ├─ No → 2D Pose Estimate 다시 찍기
  └─ Yes
       ↓
[robot-nav 실행]
       ↓
{Nav2 전부 active?}
  ├─ No → lifecycle 수동 activate
  └─ Yes
       ↓
[project_nodes 실행]
       ↓
{고정웹캠이 RC카 감지?}
  ├─ No → 현재 위치 대기
  └─ Yes
       ↓
[watch_area로 Nav2 이동]
       ↓
{도착 성공?}
  ├─ No → Nav2/costmap/goal 확인
  └─ Yes
       ↓
[OAK-D 추적 시작]
       ↓
{OAK-D가 RC카 감지?}
  ├─ No → 로봇 주변에서 대기 또는 재탐색
  └─ Yes
       ↓
[Nav2 기반 추적]
       ↓
{종료 명령?}
  ├─ No → 계속 추적
  └─ Yes → 종료
```

## 실행 순서

`project.launch.py` 한 번 실행 방식도 존재하지만, 현재 실습 환경에서는 COMM과 Nav2 lifecycle이 흔들릴 수 있음.

따라서 아래 분리 실행 방식 권장.

### 0. 공통 source

각 터미널마다 먼저 실행.

```bash
source /opt/ros/humble/setup.bash
source /etc/turtlebot4_discovery/setup.bash
source ~/mini1_ws/install/setup.bash
```

### 1. Localization 실행

터미널 1:

```bash
robot-hoon
```

RViz에서 `2D Pose Estimate`로 로봇 초기 위치 지정.

맵 확인:

```bash
ros2 topic echo --once /robot3/map
```

정상 지도 크기:

```text
width: 61
height: 77
```

### 2. Nav2 실행

터미널 2:

```bash
robot-nav
```

Nav2 상태 확인:

```bash
ros2 lifecycle get /robot3/controller_server
ros2 lifecycle get /robot3/smoother_server
ros2 lifecycle get /robot3/planner_server
ros2 lifecycle get /robot3/behavior_server
ros2 lifecycle get /robot3/bt_navigator
ros2 lifecycle get /robot3/waypoint_follower
```

정상 상태:

```text
active [3]
```

### 3. Nav2가 inactive일 때 수동 activate

`controller_server`만 active이고 나머지가 inactive일 때 사용.

```bash
ros2 lifecycle set /robot3/smoother_server activate
ros2 lifecycle set /robot3/planner_server activate
ros2 lifecycle set /robot3/behavior_server activate
ros2 lifecycle set /robot3/bt_navigator activate
ros2 lifecycle set /robot3/waypoint_follower activate
```

다시 확인:

```bash
ros2 lifecycle get /robot3/controller_server
ros2 lifecycle get /robot3/smoother_server
ros2 lifecycle get /robot3/planner_server
ros2 lifecycle get /robot3/behavior_server
ros2 lifecycle get /robot3/bt_navigator
ros2 lifecycle get /robot3/waypoint_follower
```

### 4. 프로젝트 노드 실행

터미널 3:

```bash
ros2 launch rc_car_bringup project_nodes.launch.py start_supervisor:=true start_tripod:=true start_approach:=true start_oakd:=true start_tracker:=true
```

이 명령에 포함된 기능:

```text
고정웹캠 YOLO 감지
감지 후 watch_area 이동
OAK-D YOLO + Depth 거리 계산
Nav2 기반 RC카 추적
```

### 5. 웹 ROS 연결 실행

터미널 4:

```bash
ros2 launch rc_car_bringup web_control.launch.py
```

### 6. 웹 페이지 서버 실행

터미널 5:

```bash
cd ~/mini1_ws/install/rc_car_bringup/share/rc_car_bringup/web
python3 -m http.server 8000
```

브라우저 주소:

```text
http://localhost:8000
```

브라우저가 예전 파일을 기억하면 강력 새로고침.

```text
Ctrl + F5
```

## 현재 자주 쓰는 확인 명령

고정웹캠 감지 확인:

```bash
ros2 topic echo /rc_car/webcam_detected
```

OAK-D 거리 확인:

```bash
ros2 topic echo /rc_car/target/oakd_3d | grep -E "detected|confidence|distance"
```

로봇 이동 명령 확인:

```bash
ros2 topic echo /robot3/cmd_vel
```

웹 로봇 위치 확인:

```bash
ros2 topic echo /rc_car/web/robot_pose
```

Nav2 goal 서버 확인:

```bash
ros2 action list | grep navigate_to_pose
```

TF 확인:

```bash
ros2 run tf2_ros tf2_echo map base_link --ros-args -r /tf:=/robot3/tf -r /tf_static:=/robot3/tf_static
```

## 고정 USB 웹캠 문제 해결

현재 고정 USB 웹캠 설정:

```text
/dev/video2
```

### 문제: 웹캠 포트 번호 변경

USB 웹캠은 노트북을 재부팅하거나, 카메라를 뺐다 꽂거나, 다른 카메라 앱이 먼저 카메라를 잡으면 번호가 바뀔 수 있음.

예시:

```text
어제: /dev/video7
오늘: /dev/video2
```

이 번호는 카메라의 진짜 이름이 아니라, 리눅스가 부팅할 때 임시로 붙인 번호.

따라서 `device_id`가 틀리면 YOLO 모델이 있어도 계속 `false`가 나옴.

### 올바른 웹캠 찾기

노트북 내장캠과 USB 웹캠 구분:

```bash
for d in /sys/class/video4linux/video*; do
  echo "--- $(basename $d)"
  cat "$d/name"
done
```

예시:

```text
video0
HP Wide Vision HD Camera

video2
USB Composite Device: USB Camera
```

의미:

```text
HP Wide Vision HD Camera
→ 노트북 내장캠

USB Composite Device: USB Camera
→ 고정 USB 웹캠
```

USB 웹캠 번호가 `video2`라면 `project.yaml`의 `device_id`는 `2`.

USB 웹캠 번호가 `video7`이라면 `project.yaml`의 `device_id`는 `7`.

수정 위치:

```text
src/rc_car_bringup/config/project.yaml
```

예시:

```yaml
tripod_trigger_node:
  ros__parameters:
    device_id: 2
```

수정 후 빌드:

```bash
cd ~/mini1_ws
colcon build --symlink-install --packages-select rc_car_bringup
source install/setup.bash
```

### 문제: pipewire 카메라 점유

Ubuntu 데스크톱은 `pipewire`가 카메라를 먼저 잡을 수 있음.

이때 증상:

```text
웹캠 장치 번호는 맞음
하지만 OpenCV가 카메라 열기 실패
YOLO 감지 계속 false
```

확인:

```bash
for d in /sys/class/video4linux/video*; do
  echo "--- $(basename $d)"
  cat "$d/name"
done
```

USB 카메라 예시:

```text
video2
USB Composite Device: USB Camera
```

`pipewire`가 카메라를 잡으면 OpenCV가 웹캠을 열지 못함.

점유 확인:

```bash
fuser -v /dev/video2
```

예시:

```text
/dev/video2: rokey 1769 pipewire
```

점유 해제:

```bash
kill 1769
```

OpenCV 확인:

```bash
python3 - <<'PY'
import cv2
cap = cv2.VideoCapture('/dev/video2')
print('open=', cap.isOpened())
ok, frame = cap.read()
print('read=', ok, 'shape=', None if not ok else frame.shape)
cap.release()
PY
```

정상:

```text
open= True
read= True shape= (480, 640, 3)
```

고정웹캠 노드만 단독 테스트:

```bash
ros2 launch rc_car_bringup project_nodes.launch.py start_supervisor:=false start_tripod:=true start_approach:=false start_oakd:=false start_tracker:=false
```

정상 로그:

```text
웹캠 열기 완료
웹캠 장치 번호: /dev/video2
RC카 감지 상태: True
```

## 웹 기능

웹 주소:

```text
http://localhost:8000
```

웹에서 가능한 것:

- 정적 `hoon_map` 표시
- 로봇 현재 위치 표시
- 1, 2, 3, 4번 호출 위치 표시
- 1, 2, 3, 4번 버튼으로 Nav2 이동
- Dock 버튼
- Undock 버튼

웹 지도는 `web/hoon_map.pgm` 파일을 직접 읽어서 표시.

이유:

```text
ROS map_server가 가끔 0 x 0 빈 지도를 줄 때도 웹 맵은 안정적으로 보여야 하기 때문
```

## OAK-D 거리 확인

거리값 토픽:

```bash
ros2 topic echo /rc_car/target/oakd_3d | grep -E "detected|confidence|distance"
```

예시:

```text
detected: true
confidence: 0.91
distance: 0.72
```

의미:

```text
OAK-D가 RC카를 감지함
RC카까지 거리 약 0.72m
```

## 종료 순서

가능하면 `robot-hoon`, `robot-nav`는 프로젝트 중에는 계속 켜두기.

종료 권장 순서:

```text
1. project_nodes 종료
2. web_control 종료
3. python http.server 종료
4. robot-nav 종료
5. robot-hoon 종료
```

`project.launch.py`를 한 번에 켰다가 끄면 COMM이 흔들릴 수 있어서, 현재는 분리 실행 방식 권장.

## 빌드

전체 빌드:

```bash
source /opt/ros/humble/setup.bash
source /etc/turtlebot4_discovery/setup.bash
cd ~/mini1_ws
colcon build --symlink-install
source install/setup.bash
```

자주 쓰는 부분 빌드:

```bash
cd ~/mini1_ws
colcon build --symlink-install --packages-select rc_car_bringup rc_car_follower rc_car_web
source install/setup.bash
```

## 발표용 요약

이 프로젝트는 고정웹캠, Nav2, OAK-D Depth 카메라를 나누어 사용함.

고정웹캠은 멀리서 RC카가 등장했는지만 판단.

Nav2는 로봇을 안전하게 지정 위치로 이동시킴.

OAK-D는 가까운 거리에서 RC카의 방향과 거리를 계산.

웹 페이지는 사람이 쉽게 로봇 위치와 호출 위치를 볼 수 있게 만든 화면.

로봇 제어는 `/cmd_vel`을 직접 마구 보내는 방식이 아니라, 가능한 Nav2 Action을 통해 이동하도록 구성.
