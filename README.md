## Overview
This repo contains the code for controlling both a real and a simulated differential drive robot via ROS2 using different planners, controllers, and open-source libraries for slam and odometry. 


## List of available controllers
1. Approximate linearization
2. Input-Output linearization
3. Input-Output linearization + Linear MPC via Casadi
4. Dynamic linearization
5. Nonlinear Lyapunov
6. Nonlinear MPC via Casadi
7. Nonlinear MPC via Acados
8. Iterative Linear Quadratic Regulator
9. Iterative Linear Quadratic Regulator via Crocoddyl
10. Predictive Sampling MPC


## List of available planners
1. A*
2. Breadth First search
3. Djikstra
4. Greedy Best First search
5. RRT
6. RRT* (in progress)
7. RRT-primitives


## Repository structure
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
You can use conda (experimental), or docker. For the first method, follow this readme. Otherwise, look [here](https://github.com/giulioturrisi/Differential-Drive-Robot/tree/main/installation/docker_file).

1. clone the repo
```sh
git clone --recurse-submodules https://github.com/giulioturrisi/Differential-Drive-Robot.git
```


2. install [miniforge](https://github.com/conda-forge/miniforge/releases) (x86_64) 


3. create an environment using the file in the folder [installation/conda](https://github.com/giulioturrisi/Differential-Drive-Robot/tree/master/installation/conda):

```sh
    conda env create -f mamba_environment.yml
``` 

4. follow the instruction [here](https://robostack.github.io/GettingStarted.html) to install ros-humble, and press
```sh
    mamba install ros-humble-slam-toolbox
``` 

5. download [CoppeliaSim](https://www.coppeliarobotics.com/) 

6. add in your .bashrc

```sh
alias ddrive_env="conda activate ddrive_env && source your_path_to/Differential-Drive-Robot/ros2_ws/install/setup.bash"
export COPPELIASIM_ROOT_DIR=your_path_to/CoppeliaSim
```

7. add the following ls in ros2_ws/src/simROS2/meta/interfaces.txt 
```sh
geometry_msgs/msg/Twist
geometry_msgs/msg/TwistStamped
sensor_msgs/msg/LaserScan
```

8. start your environment and go in ros2_ws
```sh
ddrive_env
cd your_path_to/Differential-Drive-Robot/ros2_ws
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro humble
ulimit -s unlimited
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

9. if you need acados, go inside the [acados](https://github.com/giulioturrisi/Differential-Drive-Robott/tree/master/python_scripts/controllers/acados)/acados folder and press
  
```sh
mkdir build
cd build
cmake -DACADOS_WITH_QPOASES=ON  -DACADOS_WITH_OSQP=ON ..
make install -j4
pip install -e ./../interfaces/acados_template
```
then in your .bashrc, add
```sh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/your_path_to/Differential-Drive-Robot/python_scripts/controllers/acados/lib"
export ACADOS_SOURCE_DIR="/your_path_to/Differential-Drive-Robot/python_scripts/controllers/acados"
```



## How to run the simulation
All the commands below can be more easily launched via some aliases, such as

```sh
alias ddrive_launch_rviz="ros2 run rviz2 rviz2 -d your_path_to/Differential-Drive-Robot/ros2_ws/src/utilities/rviz_config/common.rviz"
alias ddrive_launch_slam="ros2 launch slam_toolbox online_async_launch.py"
alias ddrive_launch_planners="ros2 run planners run_planners"
alias ddrive_launch_joy="ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'"
alias ddrive_launch_controllers="ros2 run controllers run_controllers"
alias ddrive_launch_lidar_odometry="ros2 launch pointcloud_to_laserscan sample_laserscan_to_pointcloud_launch.py | ros2 launch kiss_icp odometry.launch.py topic:=/pointcloud visualize:=false child_frame:=base_footprint max_range:=5.0 min_range:=0.2"
alias ddrive_launch_state_publisher="ros2 launch state_estimation state_publisher_launch.py "
alias ddrive_launch_sim_dynamics="cd $COPPELIASIM_ROOT_DIR && ./coppeliaSim.sh -f /your_path_to/Differential-Drive-Robot/coppeliasim_simulation/dynamics.ttt"
alias ddrive_launch_sim_kinematics="cd $COPPELIASIM_ROOT_DIR && ./coppeliaSim.sh -f your_path_to/Differential-Drive-Robot/coppeliasim_simulation/kinematics.ttt"
```

Follow the commands below to run all the framework:

1. on a new terminal first launch the simulation 
```sh
ddrive_launch_sim_kinematics                                              (scene with kinematics)
ddrive_launch_sim_dynamics                                                (scene with dynamics)
```

2. on each new terminal, then launch all the other packages 
```sh
ddrive_launch_joy                                                         (to use the joystick)
ddrive_launch_rviz                                                        (visualization)
ddrive_launch_planners                                                    (planning)
ddrive_launch_controllers                                                 (control)
ddrive_launch_lidar_odometry                                              (tf and kiss-icp)
ddrive_launch_slam                                                        (slam-toolbox)
ros2 launch ydlidar_ros2_driver ydlidar_launch.py                  (ydlidar - only real robot)
```

3. you can choose a goal pose in Rviz2 clicking 2D Goal Pose
   

## Status
Still working in progress, the real robot exists but it's not yet finalized!

