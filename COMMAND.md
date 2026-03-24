# Commands:
## To run the car with joystick:
```
cd ~/f1tenth_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_stack no_lidar_bringup_launch.py
```

## Run Foxglove Bridge:
(f1tenth_ws)    
```
cd ~/f1tenth_ws/
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

## To run the positional tracking:
(zed_ws)    
```
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run zedx_pure_pursuit fast_localization.py
```

## To run the mapping:
(zed_ws)    
```
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run zedx_pure_pursuit zedx_mapping.py
```


## To run waypoints:
(zed_ws)
```
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run zedx_pure_pursuit waypoints.py 
```
----------------------------------------------------------------------------------

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
ros2 run zedx_pure_pursuit zedx_localization_bridge.py
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