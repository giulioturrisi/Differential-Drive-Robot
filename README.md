## Overview
This repo contains the code for controlling both a real and a simulated differential drive robot via ROS2 using different planners, controllers, and open-source libraries for slam and odometry. 


## Repo structure
It includes the following folders and subfolders:

1. ```python_scripts```: most of the ROS2 nodes call some classes here
 
2. ```coppeliasim_simulation```: scenes used for simulating the robot (dynamically enabled or not)

3. ```ros2_ws```: collection of ROS2 nodes for controlling the robot with the aid of some external tools such as ```slam_toolbox``` (generate a map and localization) and ```kiss-icp``` (lidar odometry)

 
## Dependencies
1. [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)

2. [CoppeliaSim](https://www.coppeliarobotics.com/downloads)

3. [kiss-icp](https://github.com/PRBonn/kiss-icp)

4. [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)


## Build on linux
1. clone the repo
```sh
git clone --recurse-submodules https://github.com/giulioturrisi/differential_drive.git
```

2. extract CoppeliaSim in Differential-Drive-Robot/coppeliasim_simulation

3. add the following ls in ros2_ws/src/simExtROS2/meta/interfaces.txt 
```sh
geometry_msgs/msg/Twist
sensor_msgs/msg/LaserScan
```

4. build the docker file inside Differential-Drive-Robot/docker_file/integrated_gpu or /nvidia
```sh
docker build -t ros2_humble .
```

5. add alias to start the docker
```sh
cd 
gedit .bashrc
alias ddrive_humble='xhost + && docker run -it --rm -v /path/to/your_folder/Differential-Drive-Robot:/home/ -v /tmp/.X11-unix:/tmp/.X11-unix:rw --device=/dev/input/ -e DISPLAY=$DISPLAY -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY  -e QT_X11_NO_MITSHM=1 --gpus all --name ddrive_humble ros2_humble'  (if used /nvidia)
alias ddrive_humble="xhost + && docker run -it --rm -v /path/to/your_folder/Differential-Drive-Robot:/home/ -v /tmp/.X11-unix:/tmp/.X11-unix --device=/dev/dri --device=/dev/input/ -e DISPLAY=$DISPLAY -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY --name ddrive_humble  ros2_humble" (if used /integrated_gpu)
alias ddrive_humble='xhost + && docker run -it --rm -v /path/to/your_folder/Differential-Drive-Robot:/home/ -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg -v /usr/lib/wsl:/usr/lib/wsl --device=/dev/dxg -e DISPLAY=$DISPLAY -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR -e PULSE_SERVER=$PULSE_SERVER -e LD_LIBRARY_PATH=/usr/lib/wsl/lib --name ddrive_humble ros2_humble' (if Windows Linux Subsystem)

alias ddrive='docker exec -it ddrive_humble bash' (to attach a new terminal to the running docker)
```

6. start docker and build
```sh
ddrive_humble
cd ros2_ws
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro humble
ulimit -s unlimited
colcon build --symlink-install
```


## How to run the simulation
All the commands below can be easily launched via some aliases. Check them by activating the docker ```ddrive_humble``` and writing on the keyboard ```launch_``` (plus tab for the autocomplete)

Follow the commands below to run all the framework:

1. on a new terminal 
```sh
launch_sim_kinematics                                              (scene with kinematics)
launch_sim_dynamics                                                (scene with dynamics)
```

2. on each terminal you can press 
```sh
launch_joy                                                         (to use the joystick)
launch_rviz                                                        (visualization)
launch_planners                                                    (planning)
launch_controllers                                                 (control)
launch_lidar_odometry                                              (tf, robot model, and kiss-icp)
launch_slam                                                        (slam-toolbox)
ros2 launch ydlidar_ros2_driver ydlidar_launch.py                  (ydlidar - only real robot)
```
3. (optional) to load an existing map, put in slam_toolbox/congif/mapper_params_localization /home/ros2_ws/src/utilities/name_of_map


4. you can choose a goal pose in Rviz2 clicking 2D Goal Pose
   

## List of available controllers
1. Approximate linearization
2. Input-Output linearization
3. Dynamic linearization
4. Nonlinear Lyapunov
5. Nonlinear MPC via Casadi
6. Nonlinear MPC via Acados
7. Iterative Linear Quadratic Regulator
8. Iterative Linear Quadratic Regulator via Crocoddyl
9. Predictive Sampling MPC


## List of available planners
1. A*
2. Breadth First search
3. Djikstra
4. Greedy Best First search
5. RRT
6. RRT* (in progress)
7. RRT-primitives


## Status
Still working in progress, the real robot exists but it's not yet finalized!

