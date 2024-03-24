## Build on linux using docker
1. clone the repo
```sh
git clone --recurse-submodules https://github.com/giulioturrisi/differential_drive.git
```

2. extract CoppeliaSim in Differential-Drive-Robot/coppeliasim_simulation

3. add the following ls in ros2_ws/src/simExtROS2/meta/interfaces.txt 
```sh
geometry_msgs/msg/Twist
geometry_msgs/msg/TwistStamped
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

1. on a new terminal first launch the simulation 
```sh
launch_sim_kinematics                                              (scene with kinematics)
launch_sim_dynamics                                                (scene with dynamics)
```

2. on each new terminal, then launch all the other packages 
```sh
launch_joy                                                         (to use the joystick)
launch_rviz                                                        (visualization)
launch_planners                                                    (planning)
launch_controllers                                                 (control)
launch_lidar_odometry                                              (tf and kiss-icp)
launch_slam                                                        (slam-toolbox)
ros2 launch ydlidar_ros2_driver ydlidar_launch.py                  (ydlidar - only real robot)
```

3. you can choose a goal pose in Rviz2 clicking 2D Goal Pose
   
