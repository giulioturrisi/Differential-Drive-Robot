"""2D laser scanner for MuJoCo, published as sensor_msgs/LaserScan.

MuJoCo has no planar lidar, so a fan of horizontal rays is cast from a site with
mj_multiRay. Defaults follow the Hokuyo URG-04LX script of the old CoppeliaSim
scenes: topic /scan, frame base_footprint, 360 rays (1 deg), angle_min = 0.
Rays that hit nothing within range_max are published as inf.
"""
import array
import math

import mujoco
import numpy as np
from sensor_msgs.msg import LaserScan


class Lidar2D:
    def __init__(self, node, model, data, site_name="lidar", topic="scan", frame_id="base_footprint",
                 num_rays=360, angle_min=0.0, range_min=0.02, range_max=5.6, noise_std=0.0):
        self.node = node
        self.model = model
        self.data = data
        self.num_rays = num_rays
        self.range_min = range_min
        self.range_max = range_max
        self.noise_std = noise_std

        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if self.site_id < 0:
            raise ValueError("site '" + site_name + "' not found in the model")
        # The rays start inside the lidar housing: skip the body the site belongs to
        self.body_exclude = int(model.site_bodyid[self.site_id])

        # Ray directions in the site frame (x forward, counterclockwise around z)
        self.angle_increment = 2.0 * math.pi / num_rays
        self.angles = angle_min + self.angle_increment * np.arange(num_rays)
        self.local_directions = np.stack([np.cos(self.angles), np.sin(self.angles), np.zeros(num_rays)], axis=1)

        self.geom_id = np.zeros(num_rays, dtype=np.int32)
        self.distance = np.zeros(num_rays)
        self.ranges = np.full(num_rays, np.inf)
        self.hit_points = np.zeros((0, 3))
        self.use_normal_argument = True

        self.msg = LaserScan()
        self.msg.header.frame_id = frame_id
        self.msg.angle_min = float(angle_min)
        self.msg.angle_max = float(self.angles[-1])
        self.msg.angle_increment = float(self.angle_increment)
        self.msg.time_increment = 0.0
        self.msg.range_min = float(range_min)
        self.msg.range_max = float(range_max)

        self.publisher = node.create_publisher(LaserScan, topic, 1)


    # Ray casting ------------------------------------------------------
    def _multi_ray(self, origin, directions):
        # MuJoCo >= 3.3 added the "normal" output argument
        if self.use_normal_argument:
            try:
                mujoco.mj_multiRay(self.model, self.data, origin, directions, None, 1, self.body_exclude,
                                   self.geom_id, self.distance, None, self.num_rays, self.range_max)
                return
            except TypeError:
                self.use_normal_argument = False
        mujoco.mj_multiRay(self.model, self.data, origin, directions, None, 1, self.body_exclude,
                           self.geom_id, self.distance, self.num_rays, self.range_max)


    def scan(self):
        """Cast the rays at the current state (geom poses must be up to date) and return the ranges."""
        origin = self.data.site_xpos[self.site_id].copy()
        rotation = self.data.site_xmat[self.site_id].reshape(3, 3)
        directions = self.local_directions @ rotation.T
        self._multi_ray(origin, directions.flatten())

        hit = (self.geom_id >= 0) & (self.distance <= self.range_max)
        self.ranges = np.full(self.num_rays, np.inf)
        self.ranges[hit] = self.distance[hit]
        if self.noise_std > 0.0:
            self.ranges[hit] += np.random.normal(0.0, self.noise_std, np.count_nonzero(hit))
        self.hit_points = origin + directions[hit] * self.distance[hit, None]
        return self.ranges


    # Publication ------------------------------------------------------
    def publish(self, scan_time):
        """Scan and publish a LaserScan, stamped with the ROS clock as in CoppeliaSim."""
        ranges = self.scan()
        self.msg.header.stamp = self.node.get_clock().now().to_msg()
        self.msg.scan_time = float(scan_time)
        self.msg.ranges = array.array('f', ranges.astype(np.float32).tobytes())
        self.publisher.publish(self.msg)


    # Visualization ----------------------------------------------------
    def draw(self, scene, rgba=(1.0, 0.0, 0.0, 1.0), size=0.01):
        """Draw the last hit points as small spheres in a mjvScene (e.g. viewer.user_scn)."""
        scene.ngeom = 0
        rgba = np.array(rgba, dtype=np.float32)
        point_size = np.array([size, 0.0, 0.0])
        identity = np.eye(3).flatten()
        for point in self.hit_points[:scene.maxgeom]:
            mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, point_size, point, identity, rgba)
            scene.ngeom += 1
