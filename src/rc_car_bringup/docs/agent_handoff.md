# 팀원 및 AI 도구 인계 문서

이 문서는 Codex, Claude Code, 팀원이 프로젝트를 이어서 볼 때 필요한 전체 지도.

README는 실행자용 문서.

이 문서는 코드 수정자용 문서.

## 프로젝트 목표

RC카가 고정웹캠에 보이면 TurtleBot4가 지정 위치로 이동.

그 뒤 OAK-D Depth 카메라로 RC카 거리와 방향을 계산.

마지막으로 Nav2를 이용해 장애물을 피하면서 RC카 추적.

## 현재 기준 실행 방식

`project.launch.py` 한 번 실행은 가능하지만 현재 실습장에서는 COMM과 Nav2 lifecycle이 흔들릴 수 있음.

권장 방식은 분리 실행.

```text
robot-hoon
→ RViz 2D Pose Estimate
→ robot-nav
→ 필요 시 lifecycle 수동 activate
→ project_nodes.launch.py
→ web_control.launch.py
→ python http.server
```

## 패키지 책임

### rc_car_interfaces

메시지와 서비스 정의 패키지.

주요 메시지:

```text
Target2D.msg
→ 2D 화면 감지 결과

Target3D.msg
→ 3D 거리 감지 결과

SystemState.msg
→ 시스템 상태 표시용
```

주요 서비스:

```text
SetFollowState.srv
→ 추적 상태 변경용
```

### rc_car_follower

RC카 감지, 접근, 추적 노드 패키지.

이 프로젝트의 핵심 로직 위치.

### rc_car_bringup

설정, launch, 지도, 웹 정적 파일, 문서 패키지.

실행할 때 가장 많이 만지는 패키지.

### rc_car_web

웹 브라우저와 ROS2를 이어주는 패키지.

웹 버튼 명령과 로봇 위치 표시 담당.

### ros2_homework_examples

ROS2 수업 숙제용 임시 패키지.

검사 후 삭제 가능.

메인 RC카 프로젝트 실행에는 필요 없음.

## 주요 노드 설명

### tripod_trigger_node

파일:

```text
src/rc_car_follower/rc_car_follower/tripod_trigger_node.py
```

역할:

```text
노트북에 연결된 고정 USB 웹캠 열기
YOLOv8n COCO car 감지
연속 감지 기준 적용
/rc_car/webcam_detected 발행
```

입력:

```text
/dev/video2 같은 USB 웹캠 장치
```

출력:

```text
/rc_car/webcam_detected
/rc_car/debug/tripod_image
/rc_car/debug/tripod_image/compressed
```

주의:

```text
웹캠 번호는 재부팅/재연결마다 바뀔 수 있음
pipewire가 카메라를 잡으면 OpenCV가 열 수 없음
```

### approach_planner_node

파일:

```text
src/rc_car_follower/rc_car_follower/approach_planner_node.py
```

역할:

```text
/rc_car/webcam_detected가 true가 되면 watch_area waypoint로 Nav2 goal 전송
watch_area 도착 성공 시 /rc_car/nav2_tracking_enabled true 발행
```

입력:

```text
/rc_car/webcam_detected
watch_area_waypoint.yaml
```

출력:

```text
/robot3/navigate_to_pose Action goal
/rc_car/nav2_tracking_enabled
```

주의:

```text
Nav2가 active가 아니면 goal이 들어가도 로봇이 움직이지 않음
이미 true 상태에서 노드를 켜면 새 rising edge가 없어 goal이 안 나갈 수 있음
그때는 RC카를 잠깐 웹캠 밖으로 빼서 false 후 다시 true 만들기
```

### oakd_yolo_depth_node

파일:

```text
src/rc_car_follower/rc_car_follower/oakd_yolo_depth_node.py
```

역할:

```text
OAK-D RGB preview에서 YOLO car 감지
박스 중심 위치를 Depth 이미지 좌표로 변환
주변 depth 중앙값으로 거리 계산
/rc_car/target/oakd_3d 발행
```

입력:

```text
/robot3/oakd/rgb/preview/image_raw
/robot3/oakd/stereo/image_raw
```

출력:

```text
/rc_car/target/oakd_3d
/rc_car/debug/oakd_image
/rc_car/debug/oakd_image/compressed
```

중요 코드 의미:

```text
RGB 크기와 Depth 크기가 다를 수 있음
그래서 RGB 박스 중심 좌표를 Depth 크기에 맞게 비율 변환
```

### nav2_target_tracker_node

파일:

```text
src/rc_car_follower/rc_car_follower/nav2_target_tracker_node.py
```

역할:

```text
OAK-D가 본 RC카 위치를 Nav2 goal로 변환
RC카와 약 0.70m 거리를 유지하도록 목표점 생성
Nav2를 사용하므로 costmap 장애물 회피 사용 가능
```

입력:

```text
/rc_car/target/oakd_3d
/rc_car/nav2_tracking_enabled
/robot3/tf
/robot3/tf_static
```

출력:

```text
/robot3/navigate_to_pose Action goal
```

### follow_controller_node

역할:

```text
직접 /robot3/cmd_vel 발행 추적 테스트용
```

현재 최종 주행 방식은 아님.

이유:

```text
cmd_vel 직접 제어는 Nav2 costmap 장애물 회피를 쓰지 않음
최종 추적은 nav2_target_tracker_node 사용
```

### web_command_node

파일:

```text
src/rc_car_web/rc_car_web/web_command_node.py
```

역할:

```text
웹 버튼 명령 수신
1, 2, 3, 4번 코너 이동
Dock / Undock Action 호출
```

입력:

```text
/rc_car/web/command
```

출력:

```text
/robot3/navigate_to_pose
/robot3/dock
/robot3/undock
/rc_car/web/status
```

### web_pose_node

역할:

```text
로봇 위치를 웹으로 전달
TF 기반 map -> base_link 확인
TF 실패 시 /robot3/amcl_pose fallback 사용
```

출력:

```text
/rc_car/web/robot_pose
```

### web_map_node

역할:

```text
원래는 /robot3/map을 /rc_car/web/map으로 재발행
현재 웹 배경 지도는 안정성을 위해 정적 hoon_map.pgm 직접 사용
```

남겨둔 이유:

```text
나중에 ROS map 토픽 기반 웹 지도로 되돌릴 때 사용 가능
```

## 전체 시퀀스

```text
1. robot-hoon 실행
2. RViz 2D Pose Estimate
3. robot-nav 실행
4. Nav2 lifecycle active 확인
5. 필요 시 smoother/planner/behavior/bt_navigator 수동 activate
6. project_nodes.launch.py 실행
7. 고정웹캠이 RC카 감지
8. approach_planner_node가 watch_area goal 전송
9. 로봇이 watch_area로 이동
10. 도착 성공 시 nav2_tracking_enabled true
11. OAK-D가 RC카 거리 계산
12. nav2_target_tracker_node가 RC카 주변 목표점 생성
13. Nav2가 장애물을 피하면서 추적
14. 웹에서 지도, 로봇 위치, 1~4번 위치 표시
```

## 현재 자주 터지는 문제와 원인

### Nav2가 active 안 됨

증상:

```text
controller_server active
smoother_server inactive
planner_server inactive
behavior_server inactive
bt_navigator inactive
```

해결:

```bash
ros2 lifecycle set /robot3/smoother_server activate
ros2 lifecycle set /robot3/planner_server activate
ros2 lifecycle set /robot3/behavior_server activate
ros2 lifecycle set /robot3/bt_navigator activate
ros2 lifecycle set /robot3/waypoint_follower activate
```

### map frame 없음

증상:

```text
StaticLayer: "map" passed to lookupTransform argument target_frame does not exist
AMCL cannot publish a pose...
```

해결:

```text
robot-hoon 먼저 실행
RViz 2D Pose Estimate 찍기
tf2_echo map base_link 확인
그 다음 robot-nav 실행
```

확인:

```bash
ros2 run tf2_ros tf2_echo map base_link --ros-args -r /tf:=/robot3/tf -r /tf_static:=/robot3/tf_static
```

### 웹캠 감지 false 지속

원인 후보:

```text
USB 웹캠 번호 변경
노트북 내장캠을 보고 있음
pipewire가 USB 웹캠 점유
YOLO confidence 부족
```

확인:

```bash
for d in /sys/class/video4linux/video*; do
  echo "--- $(basename $d)"
  cat "$d/name"
done
```

pipewire 점유 확인:

```bash
fuser -v /dev/video2
```

점유 해제:

```bash
kill <PID>
```

### 감지는 true인데 로봇 안 움직임

확인 순서:

```bash
ros2 node list | grep approach
ros2 topic echo /rc_car/webcam_detected
ros2 lifecycle get /robot3/bt_navigator
ros2 lifecycle get /robot3/controller_server
ros2 lifecycle get /robot3/planner_server
ros2 topic echo /robot3/cmd_vel
```

가능한 원인:

```text
approach_planner_node 미실행
Nav2 inactive
웹캠 true 상태에서 노드가 늦게 켜져 rising edge 없음
goal 위치가 costmap상 막힘
```

## 수정할 때 조심할 점

```text
robot-hoon과 robot-nav는 실험 중 가능하면 끄지 않기
project_nodes와 web_control만 필요할 때 재시작
OAK-D raw 이미지를 여러 명이 동시에 보지 않기
/robot3/cmd_vel 직접 발행 노드와 Nav2 추적 노드를 동시에 쓰지 않기
```

## 발표용 쉬운 설명

```text
고정웹캠은 초인종 역할
Nav2는 길찾기 역할
OAK-D는 가까이서 RC카까지 거리 재는 눈 역할
웹은 사람이 보는 상황판 역할
```
