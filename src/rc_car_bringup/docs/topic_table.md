# ROS 2 Communication Table

| Name | Type | Publisher / Server | Subscriber / Client | Purpose |
|---|---|---|---|---|
| `/tripod/image_raw` | `sensor_msgs/Image` | tripod camera driver | `tripod_yolo_node` | Fixed camera image |
| `/rc_car/target/tripod_2d` | `rc_car_interfaces/Target2D` | `tripod_yolo_node` | `pixel_to_map_node` | RC car box in image pixels |
| `/rc_car/target/map` | `rc_car_interfaces/Target3D` | `pixel_to_map_node` | `approach_planner_node` | RC car position in map frame |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 action server | `approach_planner_node` | Move robot near RC car |
| `/oakd/rgb/preview/image_raw` | `sensor_msgs/Image` | OAK-D driver | `oakd_yolo_depth_node` | Robot camera RGB image |
| `/oakd/stereo/image_raw` | `sensor_msgs/Image` | OAK-D driver | `oakd_yolo_depth_node` | Robot depth image |
| `/rc_car/target/oakd_3d` | `rc_car_interfaces/Target3D` | `oakd_yolo_depth_node` | `follow_controller_node` | RC car position near robot |
| `/scan` | `sensor_msgs/LaserScan` | RPLIDAR | Nav2, `follow_controller_node` | Obstacle and safety check |
| `/cmd_vel` | `geometry_msgs/Twist` | `follow_controller_node` | TurtleBot4 base | Close-range following command |
| `/rc_car/set_state` | `rc_car_interfaces/SetFollowState` | `supervisor_node` | operator or script | Start, stop, or change mode |
| `/rc_car/system_state` | `rc_car_interfaces/SystemState` | `supervisor_node` | RViz or logger | Human-readable project state |
