# Custom Husky

+ This pacakge uses the ROS2 Control plugin - `libgazebo_ros2_control.so` to control the robot. The parameters can be tuned in the file - `control.yaml` at `/custom_husky/config/control.yaml`

+ The URDF folder has a `custom_husky.urdf.xacro` file which (if modified) needs to be converted into `custom_husky.urdf` using the command:
```
ros2 run xacro xacro -o custom_husky.urdf custom_husky.urdf.xacro
```
The robot_state_publisher used the static `.urdf` file instead of the `.urdf.xacro` one to publish the robot description on the `/robot_description` topic.

# Husky UGV

+ This package implements the most basic form of differential drive simulation using the `libgazebo_ros_diff_drive.so` plugin instead of the `libgazebo_ros2_control.so` plugin. It is easier to modify and deploy algorithms on. 

+ All the sensor plugins are stored in the `husky_ugv.urdf.xacro` file which later needs to be converted into the `husky_ugv.urdf` file using the command:
```
ros2 run xacro xacro -o husky_ugv.urdf husky_ugv.urdf.xacro
```
The robot_state_publisher used the static `.urdf` file instead of the `.urdf.xacro` one to publish the robot description on the `/robot_description` topic.