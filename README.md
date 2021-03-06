## Overview
This repo contains the code for controlling both a real and a simulated differential drive robot via ROS2. It includes the following folders and subfolders:

1. ```matlab_simulation``` with subfolders: ```maps``` (utility to modify an existing map), ```planners``` (generate an obstacle-free path with different methods), ```controllers``` (move the robot). Matlab is also used to easily generate the c++ code for ROS2.
 
2. ```coppeliasim_simulation``` contains the scenes used for simulating the robot (dynamically enabled or not)

3. ```ros2_ws``` with subfolders: ```slam_toolbox``` (generate a map - [slam](https://github.com/SteveMacenski/slam_toolbox)), ```navigation2``` (dependencies for slam - [nav](https://github.com/ros-planning/navigation2)), ```simExtROS2``` and ```ros2_bubble_rob```(dependencies for CoppeliaSim).

 
## Dependencies
1. [ROS2](https://docs.ros.org/en/foxy/Installation.html) Hunble

2. [CoppeliaSim](https://www.coppeliarobotics.com/downloads) for simulations (not mandatory)

2. [Acados](https://github.com/acados/acados) for one of the controllers (not mandatory)


#It is necessary to add the following corresponding `Env` variables in the `.bashrc` or `.bash_profile` the following line:
#```sh
#export COPPELIASIM_ROOT_DIR=~/path/to/coppeliaSim/folder
#```


## Build on Linux
1. clone the repo
```sh
git clone --recurse-submodules https://github.com/giulioturrisi/differential_drive.git
```
and extract CoppeliaSim in Differential-Drive-Robot/coppeliasim_simulation

2. add the following ls in ros2_ws/src/simExtROS2/meta/interfaces.txt 
```sh
geometry_msgs/msg/Twist
sensor_msgs/msg/LaserScan
```

3. build the docker file inside Differential-Drive-Robot/docker_file/integrated_gpu or /nvidia
```sh
docker build -t ros2_humble .
```

4. add alias to start the docker
```sh
cd 
gedit .bashrc
alias ddrive_humble='xhost + && docker run -it --rm -v /path/to/your_folder/Differential-Drive-Robot:/home/ -v /tmp/.X11-unix:/tmp/.X11-unix:rw -e DISPLAY=$DISPLAY -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY  -e QT_X11_NO_MITSHM=1 --gpus all --name ddrive_humble ros2_humble'  (if used /nvidia)
alias ddrive_humble="xhost + && docker run -it --rm -v /home/giulio/giulio_projects/Differential-Drive-Robot:/home/ -v /tmp/.X11-unix:/tmp/.X11-unix --device=/dev/dri -e DISPLAY=$DISPLAY -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY --name ddrive_humble  ros2_humble" (if used /integrated_gpu)
alias ddrive_humble='xhost + && docker run -it --rm -v /path/to/your_folder/Differential-Drive-Robot:/home/ -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg -v /usr/lib/wsl:/usr/lib/wsl --device=/dev/dxg -e DISPLAY=$DISPLAY -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR -e PULSE_SERVER=$PULSE_SERVER -e LD_LIBRARY_PATH=/usr/lib/wsl/lib --name ddrive_humble ros2_humble' (if Windows Linux Subsystem)

alias ddrive='docker exec -it ddrive_humble bash' (to attach a new terminal to the running docker)
```
You can add --device /dev/input/js0 to give to docker the access to the joystick. 

5. start docker and build
```sh
ddrive_humble
cd ros2_ws
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro humble
ulimit -s unlimited
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```


## How to run the simulation
1. Open Coppeliasim and run the scene `medium_walls_dynamics.ttt` in the folder coppeliasim_simulation 
```sh
./coppeliaSim.sh -s ../simple_walls_dynamics.ttt -h
```
disable -h flag to run the gui!

2. on a new terminal 
```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard               (if you want to use the keyboard)
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'   (if you want to use the joystick)
ros2 run rviz2 rviz2                                               (visualization)
ros2 run planners run_planners                                     (planning)
ros2 run controller input_output_linearization                     (control)
ros2 launch slam_toolbox localization_launch.py                    (localization)
```
in slam_toolbox/congif/mapper_params_localization you should put in map_file_name: /home/ros2_ws/name_of_map


3. you can choose a goal pose in Rviz2 clicking 2D Goal Pose


