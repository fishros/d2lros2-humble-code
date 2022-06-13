# 创建功能包


```
cd chapt4_ws/
ros2 pkg create example_action_rclpy --build-type ament_python --dependencies rclpy example_ros2_interfaces --destination-directory src --node-name action_robot_02 --maintainer-name "fishros" --maintainer-email "fishros@foxmail.com"
touch src/example_action_rclpy/example_action_rclpy/action_control_02.py
```
