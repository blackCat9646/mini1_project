# 카메라 화면에서 찾은 RC카 정보입니다.
# 예: "화면 가운데보다 오른쪽에 RC카가 있다"를 숫자로 알려줍니다.

std_msgs/Header header

# true: RC카를 찾음, false: 이번 화면에서는 못 찾음
bool detected

# 어떤 카메라가 보낸 정보인지 적습니다. 예: tripod_camera, oakd_rgb
string source

# YOLO 클래스 이름입니다. 이번 프로젝트에서는 보통 rc_car 입니다.
string class_name

# YOLO가 얼마나 확신하는지입니다. 0.0은 모름, 1.0은 매우 확실함입니다.
float32 confidence

# 추적기가 같은 물체에 붙여 주는 번호입니다. 없으면 -1을 씁니다.
int32 track_id

# RC카 박스의 중심점입니다. 단위는 픽셀입니다.
float32 center_x
float32 center_y

# RC카 박스의 가로/세로 크기입니다. 단위는 픽셀입니다.
float32 width
float32 height
