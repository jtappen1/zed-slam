# Commands:
## To run the car with joystick:
```
cd ~/f1tenth_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_stack no_lidar_bringup_launch.py
```

## To run the positional tracking and ZedX camera:
(zed_ws)    
```
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx positional_tracking:=True publish_tf:=true 
```

## Launch the Bridge Node:
(zed_ws)  
```
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run zed_pure_pursuit 
```

## Run pure pursuit: 
(zed_ws)  
```
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run zedx_pure_pursuit pure_pursuit_node
```

## Collecting waypoints and area files:
(zed_ws)  
```
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run zedx_pure_pursuit collect_waypoints.py 
```