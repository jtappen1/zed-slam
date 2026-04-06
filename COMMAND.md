# Commands:
`NOTE`: This file assumes that you have followed previous steps to set up the F1/10th system in a parallel workspace.

## To Drive the car with joystick:
`(f1tenth_ws)`    
```
cd ~/f1tenth_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_stack no_lidar_bringup_launch.py
```

## Run Foxglove Bridge:
`(f1tenth_ws)`    
```
cd ~/f1tenth_ws/
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

## To run the positional tracking (SLAM):
`(zed_ws)`    
```
cd ~/zed_ws/
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run zedx_pure_pursuit zed_slam.py --ros-args -p fps:=30 -p resolution:=SVGA -p area_file:=/home/nvidia/zed_ws/src/zedx_pure_pursuit/data/maps/test.area -p initial_mapping:=true -p update_map:=true
```
  
# Optional:
## To run the ZEDX ROS2 Wrapper:
`(zed_ws)`    
```
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx positional_tracking:=True publish_tf:=true 
```
  
# Pure Pursuit:

## Run Pure Pursuit: 
`NOTE:` The config file for Pure Pursuit is found here [config](./config/pure_pursuit.yaml)
`(zed_ws)`  
```
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 run zedx_pure_pursuit pure_pursuit_node.py
```

## To collect waypoints:
`(zed_ws)`
```
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run zedx_pure_pursuit waypoints.py 
```