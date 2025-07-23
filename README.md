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

# SLAM with Custom Husky

+ Clone and build the `rtabmap-ros` package into a separate workspace
```
mkdir -p ~/rtabmap_ws/src/
cd ~/rtabmap_ws/src/
git clone https://github.com/introlab/rtabmap_ros.git
cd .. && colcon build --symlink-install
```

+ Replace the `rtabmap.launch.py` file with the one at **/extras/launch/rtabmap.launch.py** [provided in this repo]
+ Replace the `rgbd.rviz`file with the one at **/extras/rviz/rtabmap/rgbd.rviz** [provided in this repo]

+ Launch `RTAB-Map`:
```
ros2 launch rtabmap_launch rtabmap.launch.py
```

# Autonomous Navigation

## Install Nav2 packages for ROS 2 Humble
```
sudo apt-get install ros-humble-navigation2 ros-humble-nav2-bringup
```

## Navigation after SLAM

+ Copy the map image and yaml files to **/custom_husky_nav2/maps/**
+ Launch Navigation
```
ros2 launch custom_husky_nav2 nav2.launch.py
```

## Navigation while SLAM

+ Launch Navigation while RTAB-Map SLAM is running
```
ros2 launch nav2_bringup navigation.launch.py use_sim_time:=true
```
