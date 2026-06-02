# ros2_homework_examples

ROS2 publisher/subscriber 숙제용 예제 패키지.

## 파일 목록

- `2_0_a_image_publisher.py`
- `2_0_b_image_subscriber.py`
- `2_0_c_data_publisher.py`
- `2_0_d_data_subscriber.py`

## 실행 예시

데이터 publisher/subscriber:

```bash
ros2 run ros2_homework_examples 2_0_c_data_publisher.py
ros2 run ros2_homework_examples 2_0_d_data_subscriber.py
```

이미지 publisher/subscriber:

```bash
ros2 run ros2_homework_examples 2_0_a_image_publisher.py
ros2 run ros2_homework_examples 2_0_b_image_subscriber.py
```

토픽 확인:

```bash
ros2 topic list
ros2 topic echo /homework/data
rqt_image_view /homework/image
```
