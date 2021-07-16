# Differential Drive

# Overview
 - [:orange_book: The general idea](#orange_book-some-theory-behind-the-code)
 - [:page_facing_up: Dependencies](#page_facing_up-dependencies)

# The general idea
This repo contains the code for controlling a differential drive robot via Ros2. It includes Matlab scripts for planning (to easily generate a C++ code to use inside Ros), and two CoppeliaSim simulation fully integrated with Ros.  


## Dependencies
1. *ROS2:* Build and install ROS2 Foxy [Ros2](https://docs.ros.org/en/foxy/index.html)

2. *CoppeliaSim:* For simulations [CoppeliaSim](https://www.coppeliarobotics.com/downloads)

It is necessary to add the following corresponding `Env` variables in the `.bashrc` or `.bash_profile` the following lines:
sh
export COPPELIASIM_ROOT_DIR=~/path/to/coppeliaSim/folder


See how to edit system `Env` variables in `Windows` [here](https://appuals.com/how-to-edit-environment-variables-in-windows-10) 


## Linux
1. 
sh
git clone --recurse-submodules https://github.com/giulioturrisi/differential_drive.git
cd differential_drive/ros2_ws/src
mv dc_motor_l298N ./../
mv driver_motor ./../

2. 
sh
cd src/simExtROS2/meta


add the lines geometry_msgs/msg/Twist and sensor_msgs/msg/LaserScan in interfaces.txt 

3. 
sh
cd ../../../
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy
ulimit -s unlimited
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release


## Windows
Under construction

## How to run the simulation
1. 
Open Coppeliasim and run the scene `simple_walls_dynamics.ttt` in the folder coppeliasim_simulation

2.  
open a new terminals 
sh
run ros2 run rviz2 rviz2 
ros2 run motion_planner RRT_input_output_smooth
ros2 run controller input_output_linearization

3. 
you can choose a goal pose in Rviz2 clicking 2D Goal Pose


