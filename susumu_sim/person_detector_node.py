#!/usr/bin/env python3
"""LiDAR-based moving-person detector.

Detects people purely from the 3D LiDAR point cloud (/velodyne_points) — it does
NOT read any HuNavSim ground-truth topic. People are found as person-sized point
clusters in a height band, transformed into a fixed frame, then tracked across
frames so a velocity can be estimated. Clusters that move are reported as people
("moving objects" / 動物体).

Outputs:
  /perception/persons        geometry_msgs/PoseArray  (all tracked persons, in fixed frame)
  /perception/persons/markers visualization_msgs/MarkerArray (RViz: spheres + velocity arrows)

The follow node consumes /perception/persons. Velocity per person is encoded in
the marker arrows; the follow node re-estimates heading itself from the pose
history, so PoseArray (a standard message, no custom msg) is enough.
"""

import math
from collections import deque

import numpy as np
from scipy import ndimage

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import PoseArray, Pose, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from tf2_ros import TransformException


class Track:
    """A single tracked person: smoothed position + velocity from history."""
    _next_id = 0

    def __init__(self, xy, stamp):
        self.id = Track._next_id
        Track._next_id += 1
        self.xy = np.asarray(xy, dtype=float)
        self.vel = np.zeros(2, dtype=float)
        self.last_stamp = stamp
        self.hits = 1
        self.misses = 0
        self.history = deque(maxlen=10)
        self.history.append((stamp, self.xy.copy()))

    def update(self, xy, stamp):
        xy = np.asarray(xy, dtype=float)
        # Light smoothing on position (centroid jitter is ~voxel size).
        alpha = 0.6
        new_xy = alpha * xy + (1.0 - alpha) * self.xy
        self.xy = new_xy
        self.last_stamp = stamp
        self.hits += 1
        self.misses = 0
        self.history.append((stamp, self.xy.copy()))
        # Velocity over a longer baseline (~0.5 s) so slow walking (~0.6 m/s)
        # rises above the per-frame centroid jitter.
        if len(self.history) >= 2:
            t0, p0 = self.history[0]
            t1, p1 = self.history[-1]
            base_dt = t1 - t0
            if base_dt > 0.2:
                self.vel = (p1 - p0) / base_dt

    def speed(self):
        return float(np.linalg.norm(self.vel))


class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')

        # --- parameters ---
        self.declare_parameter('input_topic', '/velodyne_points')
        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('z_min', 0.10)          # ignore floor
        self.declare_parameter('z_max', 1.90)          # ignore ceiling
        self.declare_parameter('voxel', 0.10)          # XY grid resolution [m]
        self.declare_parameter('cluster_tol', 0.30)    # dilation radius for CC [m]
        self.declare_parameter('min_points', 8)
        self.declare_parameter('person_radius_max', 0.6)   # cluster footprint radius [m]
        self.declare_parameter('person_radius_min', 0.05)
        self.declare_parameter('match_dist', 0.8)      # track association gate [m]
        self.declare_parameter('max_misses', 30)       # keep a track through brief occlusion
        self.declare_parameter('range_min', 0.45)      # drop self/near returns [m]
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('publish_only_moving', True)  # persons = moving objects
        # People walk ~0.2 m/s; keep this low so a briefly-paused person isn't
        # filtered out (which would make the follower lose its lock).
        self.declare_parameter('moving_speed', 0.04)   # speed to count as moving [m/s]
        # Radius around a moving person whose points are removed from the Nav2
        # costmap cloud (so the person leaves no obstacle trail).
        self.declare_parameter('person_clear_radius', 0.5)  # [m]

        self.input_topic = self.get_parameter('input_topic').value
        self.fixed_frame = self.get_parameter('fixed_frame').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.tracks = []

        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_cb, qos_profile_sensor_data)
        self.pub_poses = self.create_publisher(PoseArray, '/perception/persons', 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/perception/persons/markers', 10)
        # Cloud with the (moving) people removed, so Nav2's costmap does not get a
        # trail of obstacles along each person's path. Nav2's voxel_layer uses this.
        self.pub_filtered = self.create_publisher(
            PointCloud2, '/velodyne_points_filtered', qos_profile_sensor_data)

        # The 2D /scan ALSO sees the people; if Nav2's obstacle_layer used the raw
        # scan, a walking person would still leave a trail. So we republish a scan
        # with the moving people's bearings cleared -> /scan_filtered.
        self.latest_tracks_for_scan = []
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.pub_scan_filtered = self.create_publisher(
            LaserScan, '/scan_filtered', qos_profile_sensor_data)

        self.get_logger().info(
            f'person_detector up: input={self.input_topic}, fixed_frame={self.fixed_frame}')

    # ------------------------------------------------------------------
    def cloud_cb(self, msg: PointCloud2):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Look up sensor->fixed_frame transform at the cloud time.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.fixed_frame, msg.header.frame_id, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f'TF {self.fixed_frame}<-{msg.header.frame_id} unavailable: {ex}',
                                   throttle_duration_sec=2.0)
            return

        xyz_sensor = self._read_xyz(msg)          # original, sensor frame
        if xyz_sensor.shape[0] == 0:
            return
        xyz_odom = self._transform(xyz_sensor, tf)  # same points, odom frame

        # Range gate in the SENSOR frame: drop the robot's own body / immediate
        # surroundings (near) and far returns, before clustering.
        rng_min = self.get_parameter('range_min').value
        rng_max = self.get_parameter('range_max').value
        rho = np.linalg.norm(xyz_sensor[:, :2], axis=1)
        gate = (rho > rng_min) & (rho < rng_max)
        xyz = xyz_odom[gate]

        # Height band filter (drop floor + ceiling) for clustering.
        z_min = self.get_parameter('z_min').value
        z_max = self.get_parameter('z_max').value
        if xyz.shape[0] > 0:
            xyz = xyz[(xyz[:, 2] > z_min) & (xyz[:, 2] < z_max)]
        if xyz.shape[0] > 0:
            centroids = self._cluster(xyz)
            self._associate(centroids, stamp)
        self._publish(msg.header.stamp)

        # Remove the moving people from the full cloud and republish (in the
        # SENSOR frame, so Nav2 can still raytrace-clear) for the costmap, so a
        # walking person leaves no obstacle trail.
        self._publish_filtered(msg.header, xyz_sensor, xyz_odom)

    # ------------------------------------------------------------------
    def _read_xyz(self, msg):
        pts = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] == 0:
            return np.empty((0, 3))
        return np.stack([pts['x'], pts['y'], pts['z']], axis=-1).astype(np.float64)

    def _transform(self, xyz, tf):
        t = tf.transform.translation
        q = tf.transform.rotation
        # quaternion -> rotation matrix
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
            [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
        ])
        return xyz @ R.T + np.array([t.x, t.y, t.z])

    def _cluster(self, xyz):
        """2D grid connected-component clustering on the XY projection.

        Returns a list of (cx, cy) cluster centroids that look person-sized.
        """
        voxel = self.get_parameter('voxel').value
        tol = self.get_parameter('cluster_tol').value
        min_pts = self.get_parameter('min_points').value
        r_max = self.get_parameter('person_radius_max').value
        r_min = self.get_parameter('person_radius_min').value

        xy = xyz[:, :2]
        mins = xy.min(axis=0)
        idx = np.floor((xy - mins) / voxel).astype(int)
        nx = idx[:, 0].max() + 1
        ny = idx[:, 1].max() + 1
        if nx <= 0 or ny <= 0 or nx * ny > 4_000_000:
            return []

        grid = np.zeros((nx, ny), dtype=bool)
        grid[idx[:, 0], idx[:, 1]] = True

        # Dilate by the cluster tolerance so nearby cells join, then label.
        rad = max(1, int(round(tol / voxel)))
        struct = ndimage.generate_binary_structure(2, 2)
        dil = ndimage.binary_dilation(grid, structure=struct, iterations=rad)
        labels, n = ndimage.label(dil, structure=struct)
        if n == 0:
            return []

        cell_label = labels[idx[:, 0], idx[:, 1]]
        centroids = []
        for lab in range(1, n + 1):
            sel = cell_label == lab
            cnt = int(sel.sum())
            if cnt < min_pts:
                continue
            pts = xy[sel]
            c = pts.mean(axis=0)
            # footprint radius (95th percentile distance from centroid)
            d = np.linalg.norm(pts - c, axis=1)
            r = float(np.percentile(d, 95))
            if r_min <= r <= r_max:
                centroids.append(c)
        return centroids

    def _associate(self, centroids, stamp):
        gate = self.get_parameter('match_dist').value
        max_misses = self.get_parameter('max_misses').value

        unmatched = list(range(len(centroids)))
        # Greedy nearest-neighbour association.
        for tr in self.tracks:
            best, best_d = -1, gate
            for j in unmatched:
                d = float(np.linalg.norm(tr.xy - centroids[j]))
                if d < best_d:
                    best, best_d = j, d
            if best >= 0:
                tr.update(centroids[best], stamp)
                unmatched.remove(best)
            else:
                tr.misses += 1

        for j in unmatched:
            self.tracks.append(Track(centroids[j], stamp))

        self.tracks = [t for t in self.tracks if t.misses <= max_misses]

    def _publish(self, stamp):
        pa = PoseArray()
        pa.header.frame_id = self.fixed_frame
        pa.header.stamp = stamp

        markers = MarkerArray()
        # Clear previous markers.
        clear = Marker()
        clear.header.frame_id = self.fixed_frame
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        only_moving = self.get_parameter('publish_only_moving').value
        moving_speed = self.get_parameter('moving_speed').value

        for tr in self.tracks:
            if tr.hits < 3:
                continue
            moving = tr.speed() > moving_speed
            # /perception/persons = moving objects (動物体) = people. Static
            # clusters (walls/furniture/self) are excluded so the follower never
            # locks onto them. Markers below still show everything for debugging.
            if not only_moving or moving:
                p = Pose()
                p.position = Point(x=float(tr.xy[0]), y=float(tr.xy[1]), z=0.9)
                # Encode heading (from velocity) in orientation for consumers.
                yaw = math.atan2(tr.vel[1], tr.vel[0]) if tr.speed() > 0.05 else 0.0
                p.orientation.z = math.sin(yaw / 2.0)
                p.orientation.w = math.cos(yaw / 2.0)
                pa.poses.append(p)

            sphere = Marker()
            sphere.header.frame_id = self.fixed_frame
            sphere.header.stamp = stamp
            sphere.ns = 'persons'
            sphere.id = tr.id * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = Point(x=float(tr.xy[0]), y=float(tr.xy[1]), z=0.9)
            sphere.scale = Vector3(x=0.4, y=0.4, z=1.6)
            moving = tr.speed() > 0.15
            sphere.color.r = 1.0 if moving else 0.4
            sphere.color.g = 0.2 if moving else 1.0
            sphere.color.b = 0.2
            sphere.color.a = 0.6
            markers.markers.append(sphere)

            arrow = Marker()
            arrow.header.frame_id = self.fixed_frame
            arrow.header.stamp = stamp
            arrow.ns = 'persons_vel'
            arrow.id = tr.id * 2 + 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            start = Point(x=float(tr.xy[0]), y=float(tr.xy[1]), z=0.9)
            end = Point(x=float(tr.xy[0] + tr.vel[0]),
                        y=float(tr.xy[1] + tr.vel[1]), z=0.9)
            arrow.points = [start, end]
            arrow.scale = Vector3(x=0.05, y=0.1, z=0.1)
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0.1, 0.4, 1.0, 0.9
            markers.markers.append(arrow)

        self.pub_poses.publish(pa)
        self.pub_markers.publish(markers)

    def _publish_filtered(self, header, xyz_sensor, xyz_odom):
        """Republish the cloud (sensor frame) with moving people removed.

        A point is dropped when it falls within `person_clear_radius` (XY) of any
        moving track, using the odom-frame positions for the test. The output stays
        in the original sensor frame so Nav2 can still raytrace-clear stale marks.
        Walls/static structure are kept, so real obstacles are still avoided.
        """
        clear_radius = self.get_parameter('person_clear_radius').value
        moving_speed = self.get_parameter('moving_speed').value

        keep = np.ones(xyz_odom.shape[0], dtype=bool)
        for tr in self.tracks:
            if tr.hits < 3 or tr.speed() <= moving_speed:
                continue
            d = np.linalg.norm(xyz_odom[:, :2] - tr.xy, axis=1)
            keep &= d > clear_radius

        out = xyz_sensor[keep]
        out_msg = pc2.create_cloud_xyz32(header, out.astype(np.float32))
        self.pub_filtered.publish(out_msg)  # keeps original sensor frame_id

    def scan_cb(self, msg: LaserScan):
        """Republish /scan with moving people removed -> /scan_filtered.

        For each beam, the hit point is transformed into odom and, if it falls
        within `person_clear_radius` of any moving track, its range is set to +inf
        (no return). This stops the 2D obstacle_layer from leaving a trail along a
        walking person's path. Static structure (walls) is untouched.
        """
        clear_radius = self.get_parameter('person_clear_radius').value
        moving_speed = self.get_parameter('moving_speed').value

        movers = [tr.xy for tr in self.tracks
                  if tr.hits >= 3 and tr.speed() > moving_speed]

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.intensities = msg.intensities

        ranges = np.asarray(msg.ranges, dtype=np.float64)
        if not movers:
            out.ranges = msg.ranges
            self.pub_scan_filtered.publish(out)
            return

        # Transform the beam hit-points (scan frame) into odom.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.fixed_frame, msg.header.frame_id, rclpy.time.Time())
        except TransformException:
            out.ranges = msg.ranges
            self.pub_scan_filtered.publish(out)
            return

        n = ranges.shape[0]
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        pts = np.zeros((n, 3))
        pts[:, 0] = np.where(valid, ranges * np.cos(angles), 0.0)
        pts[:, 1] = np.where(valid, ranges * np.sin(angles), 0.0)
        pts_odom = self._transform(pts, tf)

        clear = np.zeros(n, dtype=bool)
        for mxy in movers:
            d = np.linalg.norm(pts_odom[:, :2] - mxy, axis=1)
            clear |= (d < clear_radius) & valid

        ranges_out = ranges.copy()
        ranges_out[clear] = float('inf')
        out.ranges = ranges_out.astype(np.float32).tolist()
        self.pub_scan_filtered.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
