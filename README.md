## Overview
This repo contains the code for controlling both a real and a simulated differential drive robot via ROS2. It includes the following folders and subfolders:

1. ```matlab_simulation``` with subfolders: ```maps``` (utility to modify an existing map), ```planners``` (generate an obstacle-free path with different methods), ```controllers``` (move the robot). Matlab is also used to easily generate the c++ code for ROS2.
 
2. ```coppeliasim_simulatio``` contains the scenes used for simulating the robot (dynamically enabled or not)

3. ```ros2_ws``` with subfolders: ```slam_toolbox``` (generate a map - [slam](https://github.com/SteveMacenski/slam_toolbox)), ```navigation2``` (dependencies for slam - [nav](https://github.com/ros-planning/navigation2)), ```simExtROS2``` and ```ros2_bubble_rob```(dependencies for CoppeliaSim).

 
## Dependencies
1. [ROS2](https://docs.ros.org/en/foxy/Installation.html) Foxy

2. [CoppeliaSim](https://www.coppeliarobotics.com/downloads) for simulations (not mandatory)

2. [Acados](https://github.com/acados/acados) for one of the controllers (not mandatory)

It is necessary to add the following corresponding `Env` variables in the `.bashrc` or `.bash_profile` the following line:
```sh
export COPPELIASIM_ROOT_DIR=~/path/to/coppeliaSim/folder
```


## Build on Linux
1. 
```sh
git clone --recurse-submodules https://github.com/giulioturrisi/differential_drive.git
cd differential_drive/ros2_ws/src
```

2. 
```sh
cd src/simExtROS2/meta
```
add the lines geometry_msgs/msg/Twist and sensor_msgs/msg/LaserScan in interfaces.txt 

3. 
```sh
cd ../../../
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy
ulimit -s unlimited
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```


## How to run the simulation
1. Open Coppeliasim and run the scene `simple_walls_dynamics.ttt` in the folder coppeliasim_simulation

2. open a new terminals 
```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard               (if you want to use the keyboard)
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'   (if you want to use the joystick)
ros2 run rviz2 rviz2                                               (visualization)
ros2 run motion_planner RRT_input_output_smooth                    (planning)
ros2 run controller input_output_linearization                     (control)
ros2 launch slam_toolbox localization_launch.py                    (localization)
```
in slam_toolbox/congif/mapper_params_localization you should put in map_file_name: simple_walls_map


3. you can choose a goal pose in Rviz2 clicking 2D Goal Pose


