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
6. RRT*
7. Informed RRT*
8. RRT-primitives


## Repository structure
It includes the following folders and subfolders:

1. ```python_scripts```: most of the ROS2 nodes call some classes here
 
2. ```mujoco_simulation```: MuJoCo model of the robot (generated from the urdf) and of the environment, plus the script that simulates it via ROS2 (dynamically enabled or not, with a 2D lidar)

3. ```ros2_ws```: collection of ROS2 nodes for controlling the robot with the aid of some external tools such as ```slam_toolbox``` (generate a map and localization) and ```kiss-icp``` (lidar odometry)

4. ```installation```: [pixi](https://pixi.sh) workspace (```pixi.toml``` and ```pixi.lock```) with all the dependencies and the commands to build and run everything

 
## Dependencies
All the dependencies are handled by [pixi](https://pixi.sh) (Linux x86_64 only):

1. [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) and [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) via [RoboStack](https://robostack.github.io)

2. [MuJoCo](https://github.com/google-deepmind/mujoco), casadi, jax, numba, crocoddyl via conda-forge

3. [acados](https://github.com/acados/acados), [kiss-icp](https://github.com/PRBonn/kiss-icp) and [pointcloud_to_laserscan](https://github.com/giulioturrisi/pointcloud_to_laserscan) are git submodules compiled from source

## Build on linux 
1. install pixi
```sh
curl -fsSL https://pixi.sh/install.sh | sh
```

2. clone the repo (if you already cloned it without submodules, run ```git submodule update --init --recursive```)
```sh
git clone --recurse-submodules https://github.com/giulioturrisi/Differential-Drive-Robot.git
```

3. install and activate the environment (the first time it downloads all the dependencies), then build acados and the ROS2 workspace
```sh
cd Differential-Drive-Robot/installation
pixi shell                                                         (or pixi shell -e nvidia with an NVIDIA GPU, jax with CUDA)
pixi run build
```
```pixi run clean``` removes all the build artifacts


## How to run the simulation
All the commands below are pixi tasks defined in ```installation/pixi.toml``` (```pixi task list``` shows them). On each new terminal activate the environment once
```sh
cd Differential-Drive-Robot/installation
pixi shell                                                         (or pixi shell -e nvidia)
```
then the tasks can be launched from any folder with ```pixi run <task>```, in the activated environment. Without activating the environment, use ```pixi run -m installation <task>``` from the root of the repo (adding ```-e nvidia``` for the nvidia environment).

Follow the commands below to run all the framework:

1. on a new terminal first launch the simulation 
```sh
pixi run launch_sim_kinematics                                     (scene with kinematics)
pixi run launch_sim_dynamics                                       (scene with dynamics)
```
they run ```python mujoco_simulation/run_simulation.py --mode kinematics/dynamics```, extra arguments are forwarded (e.g. ```pixi run launch_sim_kinematics --headless``` to run without the viewer, ```--lidar_rate``` to change the lidar frequency)

2. on each new terminal, then launch all the other packages 
```sh
pixi run launch_joy                                                (to use the joystick)
pixi run launch_rviz                                               (visualization)
pixi run launch_planners                                           (planning)
pixi run launch_controllers                                        (control)
pixi run launch_lidar_odometry                                     (tf and kiss-icp)
pixi run launch_slam                                               (slam-toolbox)
pixi run launch_state_publisher                                    (filter for encoder and lidar)
pixi run launch_plotjuggler                                        (plotting)
ros2 launch ydlidar_ros2_driver ydlidar_launch.py                  (ydlidar - only real robot, not included in the pixi environment)
```

3. you can choose a goal pose in Rviz2 clicking 2D Goal Pose
   




## Status
Still working in progress, the real robot exists but it's not yet finalized!

