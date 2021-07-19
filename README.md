## Overview
This repo contains the code for controlling both a real and a simulated differential drive robot via Ros2. It includes the following packages:

1. matlab_simulation contains ```inflate_map.m``` to inflate an existing map with the size of the robot, ```RRT_input_output_deltaInput.m``` to generate a path for a simple integrator, ```RRT_primitives.m``` to generate a path considering the kinematic of the differential drive, ```main.m``` to test the planners and controllers.
 
2. coppeliasim_simulation contains the scenes used for simulating the robot (dynamically enabled or not)

3. ros2_ws contains ```motion_planner``` that generates a path to a goal, ```controller``` to make the robot follow the given path, ```slam_toolbox``` to generate a map ([slam](https://github.com/SteveMacenski/slam_toolbox)), ```navigation2``` which contains some dependencies for slam ([nav](https://github.com/ros-planning/navigation2)), ```simExtROS2``` and ```ros2_bubble_rob``` which are used to use Ros2 in CoppeliaSim.

 
## Dependencies
1. [ROS2](https://docs.ros.org/en/foxy/Installation.html) Foxy

2. [CoppeliaSim](https://www.coppeliarobotics.com/downloads) for simulations

It is necessary to add the following corresponding `Env` variables in the `.bashrc` or `.bash_profile` the following line:
```sh
export COPPELIASIM_ROOT_DIR=~/path/to/coppeliaSim/folder
```


## Build on Linux
1. 
```sh
git clone --recurse-submodules https://github.com/giulioturrisi/differential_drive.git
cd differential_drive/ros2_ws/src
mv driver_motor ./../
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
ros2 run teleop_twist_keyboard teleop_twist_keyboard
run ros2 run rviz2 rviz2 
ros2 run motion_planner RRT_input_output_smooth
ros2 run controller input_output_linearization
```

3. you can choose a goal pose in Rviz2 clicking 2D Goal Pose


