# mini1_ws

TurtleBot4 RC car following mini project workspace.

## Workspace Idea

- `turtlebot4_ws` is the underlay workspace with the robot base packages.
- `mini1_ws` is the overlay workspace with only this mini project.

## Packages

- `rc_car_interfaces`: custom messages and service.
- `rc_car_follower`: Python ROS 2 nodes.
- `rc_car_bringup`: launch files, config files, diagrams, and YOLO dataset template.

## Build

```bash
source /opt/ros/humble/setup.bash
source /home/rokey/turtlebot4_ws/install/setup.bash
cd /home/rokey/mini1_ws
colcon build --symlink-install
source install/setup.bash
```

## Launch

```bash
ros2 launch rc_car_bringup project.launch.py
```

## YOLO Training Start Point

Collect RC car images, label them with LabelImg in YOLO format, then train:

```bash
yolo detect train model=yolov8n.pt data=/home/rokey/mini1_ws/src/rc_car_bringup/datasets/rc_car/data.yaml imgsz=640 epochs=80
```
