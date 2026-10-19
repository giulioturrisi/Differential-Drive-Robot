"""MuJoCo + ROS2 simulation of the differential drive robot.

It replaces the CoppeliaSim scenes keeping the same ROS2 interface:
    /control_vel         geometry_msgs/Twist          (sub) v, w commanded by the controllers
    /wheel_odometry      geometry_msgs/TwistStamped   (pub) v, w reconstructed from the wheel encoders
    /scan                sensor_msgs/LaserScan        (pub) 2D lidar, see lidar_2d.py
    /enableSyncMode      std_msgs/Bool                (sub) step only when /triggerNextStep arrives
    /triggerNextStep     std_msgs/Bool                (sub)
    /simulationStepDone  std_msgs/Bool                (pub) after every simulation step
    /simulationTime      std_msgs/Float32             (pub)

Usage:
    python3 run_simulation.py --mode dynamics      (wheels driven by velocity motors, physics on)
    python3 run_simulation.py --mode kinematics    (unicycle model integrated, no physics)
"""
import argparse
import math
import os
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np

import rclpy  # type: ignore
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor  # type: ignore
from rclpy.node import Node  # type: ignore
from builtin_interfaces.msg import Time  # type: ignore
from geometry_msgs.msg import Twist, TwistStamped  # type: ignore
from std_msgs.msg import Bool, Float32  # type: ignore

from lidar_2d import Lidar2D

dir_path = os.path.dirname(os.path.realpath(__file__))



class MujocoSimulation(Node):
    def __init__(self, mode, scene_path, lidar_rate):
        super().__init__('mujoco_simulation')
        self.mode = mode
        self.dt = 0.01  # same step of the CoppeliaSim scenes

        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)
        self.n_substeps = max(1, round(self.dt / self.model.opt.timestep))


        # Robot handles ------------------------------------------------
        self.root_qpos = self.model.joint('root').qposadr[0]
        self.wheels_qpos = np.array([self.model.joint('joint_left').qposadr[0], self.model.joint('joint_right').qposadr[0]])
        self.motors = np.array([self.model.actuator('motor_left').id, self.model.actuator('motor_right').id])

        # Lateral position of the wheels in base_footprint (in the URDF "wheel_left" is the one at negative y)
        self.wheels_y = np.array([self.model.body('wheel_left').pos[1], self.model.body('wheel_right').pos[1]])
        self.wheel_radius = self.model.geom('wheel_left').size[0]
        mujoco.mj_forward(self.model, self.data)


        # Publishers and Subscribers -----------------------------------
        self.command = (0.0, 0.0)
        self.create_subscription(Twist, 'control_vel', self.control_vel_callback, 1)
        self.publisher_wheel_odometry = self.create_publisher(TwistStamped, 'wheel_odometry', 1)

        self.lidar = Lidar2D(self, self.model, self.data)
        self.lidar_every = max(1, round(1.0 / (lidar_rate * self.dt)))
        self.step_count = 0


        # Synchronization with the controllers -------------------------
        self.sync_mode = False
        self.step_trigger = threading.Event()
        self.create_subscription(Bool, 'enableSyncMode', self.enable_sync_mode_callback, 1)
        self.create_subscription(Bool, 'triggerNextStep', self.trigger_next_step_callback, 1)
        self.publisher_step_done = self.create_publisher(Bool, 'simulationStepDone', 1)
        self.publisher_simulation_time = self.create_publisher(Float32, 'simulationTime', 1)

        self.get_logger().info("MuJoCo simulation started in " + mode + " mode")


    # Callbacks ---------------------------------------------------------
    def control_vel_callback(self, msg):
        self.command = (msg.linear.x, msg.angular.z)

    def enable_sync_mode_callback(self, msg):
        self.sync_mode = msg.data

    def trigger_next_step_callback(self, msg):
        self.step_trigger.set()


    # Robot model -------------------------------------------------------
    def wheel_speeds(self, v, w):
        # Rolling without slipping: the wheel at lateral position y moves at v - w*y
        return (v - w * self.wheels_y) / self.wheel_radius

    def kinematic_step(self, v, w):
        qpos = self.data.qpos
        quat = qpos[self.root_qpos + 3:self.root_qpos + 7]
        theta = math.atan2(2.0 * (quat[0] * quat[3] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] ** 2 + quat[3] ** 2))

        qpos[self.root_qpos] += v * math.cos(theta) * self.dt
        qpos[self.root_qpos + 1] += v * math.sin(theta) * self.dt
        theta += w * self.dt
        qpos[self.root_qpos + 3:self.root_qpos + 7] = [math.cos(theta / 2.0), 0.0, 0.0, math.sin(theta / 2.0)]
        qpos[self.wheels_qpos] += self.wheel_speeds(v, w) * self.dt

        self.data.qvel[:] = 0.0
        self.data.time += self.dt
        mujoco.mj_forward(self.model, self.data)

    def dynamic_step(self, v, w):
        self.data.ctrl[self.motors] = self.wheel_speeds(v, w)
        for _ in range(self.n_substeps):
            mujoco.mj_step(self.model, self.data)


    # Simulation step ---------------------------------------------------
    def step(self):
        v, w = self.command
        wheels_before = self.data.qpos[self.wheels_qpos].copy()

        if self.mode == 'kinematics':
            self.kinematic_step(v, w)
        else:
            self.dynamic_step(v, w)

        # Wheel odometry from the encoders, stamped with the simulation time as in CoppeliaSim
        wheels_speed = (self.data.qpos[self.wheels_qpos] - wheels_before) / self.dt
        w_measured = self.wheel_radius * (wheels_speed[0] - wheels_speed[1]) / (self.wheels_y[1] - self.wheels_y[0])
        v_measured = self.wheel_radius * wheels_speed[0] + w_measured * self.wheels_y[0]

        odometry = TwistStamped()
        odometry.header.stamp = Time(sec=int(self.data.time), nanosec=int((self.data.time % 1.0) * 1e9))
        odometry.header.frame_id = 'base_footprint'
        odometry.twist.linear.x = float(v_measured)
        odometry.twist.angular.z = float(w_measured)
        self.publisher_wheel_odometry.publish(odometry)

        if self.step_count % self.lidar_every == 0:
            self.lidar.publish(scan_time=self.lidar_every * self.dt)
        self.step_count += 1

        self.publisher_simulation_time.publish(Float32(data=float(self.data.time)))
        self.publisher_step_done.publish(Bool(data=True))


    # Main loop ---------------------------------------------------------
    def wait_step_request(self, viewer):
        while self.sync_mode and not self.step_trigger.is_set():
            if not rclpy.ok() or (viewer is not None and not viewer.is_running()):
                return
            self.step_trigger.wait(0.01)
            if viewer is not None:
                viewer.sync()
        self.step_trigger.clear()

    def run(self, viewer=None):
        if viewer is not None:
            viewer.cam.lookat[:] = [0.0, 0.0, 0.0]
            viewer.cam.distance = 4.5

        while rclpy.ok() and (viewer is None or viewer.is_running()):
            start_time = time.time()
            self.wait_step_request(viewer)

            if viewer is None:
                self.step()
            else:
                with viewer.lock():
                    self.step()
                    self.lidar.draw(viewer.user_scn)
                viewer.sync()

            if not self.sync_mode:
                time.sleep(max(0.0, self.dt - (time.time() - start_time)))



def main():
    parser = argparse.ArgumentParser(description="MuJoCo simulation of the differential drive robot")
    parser.add_argument('--mode', choices=['dynamics', 'kinematics'], default='dynamics')
    parser.add_argument('--scene', default=os.path.join(dir_path, 'scene.xml'))
    parser.add_argument('--lidar_rate', type=float, default=100.0, help="lidar frequency [Hz]")
    parser.add_argument('--headless', action='store_true', help="run without the viewer")
    args = parser.parse_args()

    rclpy.init()
    simulation_node = MujocoSimulation(args.mode, args.scene, args.lidar_rate)

    executor = SingleThreadedExecutor()
    executor.add_node(simulation_node)

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass
    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    try:
        if args.headless:
            simulation_node.run()
        else:
            with mujoco.viewer.launch_passive(simulation_node.model, simulation_node.data) as viewer:
                simulation_node.run(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        spin_thread.join()
        simulation_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
