# ROS2 Web Control 구조

## 목표

웹 브라우저에서 TurtleBot4를 호출하는 구조.

- 웹 지도 표시
- 로봇 현재 위치 표시
- 1, 2, 3, 4번 코너 이동 버튼
- Dock 버튼
- Undock 버튼

## 핵심 구조

```text
웹 브라우저
  ↓ rosbridge websocket
rosbridge_server
  ↓ ROS2 토픽
web_command_node
  ↓ ROS2 액션
Nav2 / Dock / Undock
  ↓
TurtleBot4
```

## 파일 구조

```text
rc_car_web/
  rc_car_web/
    web_command_node.py
    web_pose_node.py

rc_car_bringup/
  config/
    web_corners.yaml
  launch/
    web_control.launch.py
  web/
    index.html
```

## 노드 역할

### web_command_node

웹 버튼 명령을 로봇 액션으로 바꾸는 노드.

- `/rc_car/web/command` 구독
- `/rc_car/web/status` 발행
- `/robot3/navigate_to_pose` 액션 호출
- `/robot3/dock` 액션 호출
- `/robot3/undock` 액션 호출

### web_pose_node

로봇 위치를 웹 표시용 토픽으로 바꾸는 노드.

- `map -> base_link` TF 확인
- `/rc_car/web/robot_pose` 발행

## 웹 버튼 명령

```text
corner_1 → 1번 코너 이동
corner_2 → 2번 코너 이동
corner_3 → 3번 코너 이동
corner_4 → 4번 코너 이동
dock     → Dock 액션 호출
undock   → Undock 액션 호출
```

## Dock / Undock alias 대응

기존 alias:

```bash
robot-dock='ros2 action send_goal /robot3/dock irobot_create_msgs/action/Dock "{}"'
robot-undock='ros2 action send_goal /robot3/undock irobot_create_msgs/action/Undock "{}"'
```

웹 버튼:

```text
Dock 버튼   → /robot3/dock 액션 호출
Undock 버튼 → /robot3/undock 액션 호출
```

즉, 웹 버튼은 alias와 같은 액션을 호출하는 구조.

## 실행 순서

### 1. rosbridge 설치

한 번만 설치.

```bash
sudo apt install ros-humble-rosbridge-server
```

### 2. 로봇 Nav2 실행

기존 방식 사용.

```bash
robot-hoon
robot-nav
```

### 3. 웹 제어 노드 실행

```bash
source ~/mini1_ws/install/setup.bash
ros2 launch rc_car_bringup web_control.launch.py
```

### 4. 웹 페이지 실행

```bash
cd ~/mini1_ws/install/rc_car_bringup/share/rc_car_bringup/web
python3 -m http.server 8000
```

브라우저 주소:

```text
http://localhost:8000
```

## 좌표 수정 위치

1, 2, 3, 4번 코너 좌표 파일:

```text
~/mini1_ws/src/rc_car_bringup/config/web_corners.yaml
```

RViz에서 좌표를 찍은 뒤 `x`, `y`, `yaw` 값을 바꾸면 됨.
