# Build
```bash
colcon build
```

# Run
In one terminal, source the workspace and run the rviz node:
```bash
source install/setup.bash && ros2 run rviz2 rviz2 -d rviz_configs/eval_assignment_2.rviz
```

In another terminal, source the workspace and run the robot simulation:
```bash
source install/setup.bash && ros2 run rp_eval test_controller_ca --ros-args -p "base_link_ns":="robot_1" -p "laser_ns":="laser_1" -p "image":="assets/moon.png" -p resolution:="0.005"
```