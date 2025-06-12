## traffic_light_control  

라인 따라서 전진  
빨강/노랑 불일 때는 스탑  
초록불에서 다시 이동  


## 실행 방법 (최종)  
```bash
ros2 launch turtlebot3_gazebo turtlebot3_autorace_2020.launch.py
ros2 launch turtlebot3_autorace_camera intrinsic_camera_calibration.launch.py
ros2 launch turtlebot3_autorace_camera extrinsic_camera_calibration.launch.py

# 라인 검출
ros2 launch turtlebot3_autorace_detect detect_lane.launch.py

# 신호등
ros2 run turtlebot3_autorace_detect detect_tl_line
ros2 launch turtlebot3_autorace_detect detect_traffic_light.launch.py

# 차단바
ros2 launch turtlebot3_autorace_detect detect_level_crossing.launch.py

# yolo 사인 검출(미완성)
ros2 run turtlebot3_autorace_detect detect_signs_yolo

# 로봇 제어(마지막에 실행)
ros2 launch turtlebot3_autorace_mission control_lane.launch.py

# 사인 검출 더미노드
ros2 run turtlebot3_autorace_detect test --ros-args -p sign_label:=left_turn -p delay:=5.0
```
