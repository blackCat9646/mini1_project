# 지도 또는 로봇 기준으로 바꾼 RC카 위치입니다.
# 예: "로봇 앞 1.2m, 왼쪽 0.1m에 RC카가 있다"를 알려줍니다.

std_msgs/Header header

# true: 위치를 믿을 수 있음, false: 이번에는 위치를 못 구함
bool detected

# 어떤 노드가 만든 위치인지 적습니다. 예: tripod_mapper, oakd_depth
string source

# YOLO 클래스 이름입니다. 이번 프로젝트에서는 보통 rc_car 입니다.
string class_name

# 탐지 신뢰도입니다.
float32 confidence

# 추적 번호입니다. 없으면 -1을 씁니다.
int32 track_id

# frame_id가 map이면 지도 좌표, base_link이면 로봇 기준 좌표입니다.
geometry_msgs/Point position

# 로봇 또는 카메라에서 RC카까지의 거리입니다. 모르면 -1.0을 씁니다.
float32 distance

# RC카가 화면 중심에서 얼마나 벗어났는지입니다.
# 왼쪽이면 양수, 오른쪽이면 음수처럼 노드에서 규칙을 정해 씁니다.
float32 yaw_error
