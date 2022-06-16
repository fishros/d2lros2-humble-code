# 创建功能包


```
mkdir -p chapt4/chapt4_ws/src
cd chapt4/chapt4_ws/src
ros2 pkg create robot_startup --build-type ament_cmake --destination-directory src
mkdir -p src/robot_startup/launch
touch src/robot_startup/launch/example_action.launch.py
```
