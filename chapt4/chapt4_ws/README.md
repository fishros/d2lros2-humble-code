# 创建功能包


```
cd chapt4_ws/
ros2 pkg create example_action_rclpy --build-type ament_python --dependencies rclpy example_ros2_interfaces --destination-directory src --node-name action_robot_02 --maintainer-name "fishros" --maintainer-email "fishros@foxmail.com"
touch src/example_action_rclpy/example_action_rclpy/action_control_02.py
```


```
cd chapt4_ws/
ros2 pkg create robot_control_interfaces --build-type ament_cmake --destination-directory src --maintainer-name "fishros" --maintainer-email "fishros@foxmail.com"
```

```
cd chapt4_ws/
ros2 pkg create example_action_rclcpp --build-type ament_cmake --dependencies rclpy rclcpp_action robot_control_interfaces --destination-directory src --node-name action_robot_01 --maintainer-name "fishros" --maintainer-email "fishros@foxmail.com"
touch src/example_action_rclcpp/src/action_control_01.cpp
```

