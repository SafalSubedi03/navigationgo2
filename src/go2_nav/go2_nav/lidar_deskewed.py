#!/usr/bin/env python3
"""
lidar_deskew_node.py

Motion-compensates ("deskews") a Livox point cloud using a buffered,
interpolated odometry stream.

WHY: Livox Mid-360 accumulates points continuously over the aggregation
window between published frames (governed by publish_freq in your
MID360_config.json, typically ~100ms at 10Hz). If the robot's yaw, pitch,
roll, or position change during that window -- which happens constantly on
a walking quadruped, not just during turns -- each point in the resulting
cloud was actually captured from a slightly different pose. The result is
smeared/curved walls and duplicated edges that hurt RTAB-Map ICP matching
and Nav2 costmap quality.

APPROACH:
  1. Buffer incoming Odometry messages (position + orientation quaternion).
  2. For each incoming cloud, bin points into N time buckets spanning the
     cloud's [min_timestamp, max_timestamp].
  3. Interpolate (SLERP for rotation, LERP for translation) the robot pose
     at each bucket's center time from the odom buffer.
  4. Transform each bucket's points from "world at capture time" into
     "world at the cloud's reference time" (default: the last point's
     time, matching the cloud's header stamp convention), undoing the
     motion that happened during the scan.

SAFETY / FAIL-OPEN BEHAVIOR:
  - If the input cloud has no 'timestamp' field (e.g. Gazebo's lidar
    plugin, which publishes a single-stamp snapshot with no per-point
    timing), the node republishes the cloud unmodified. This means the
    exact same node/launch args are safe to include in both the
    hardwaressh and simulation graphs -- it's a no-op in sim.
  - If per-point timestamps are present but degenerate (all identical --
    a known livox_ros_driver2 issue on some xfer_format/PointCloud2
    configs, see Livox-SDK/livox_ros_driver2 issue #182), the node warns
    once and republishes unmodified rather than silently doing nothing
    useful. If you hit this, switch to the CustomMsg topic (which has a
    reliable per-point offset_time) and convert to PointCloud2 yourself,
    or update the driver.

IMPORTANT: Feed this node odometry from the robot's own proprioceptive
state estimator (e.g. /utlidar/robot_odom_restamped), NOT from RTAB-Map's
own ICP/visual odometry output. Deskewing the cloud using a pose that was
itself derived from matching that same cloud creates a circular
dependency and will not converge cleanly.

NOTE ON FRAMES: this applies the odometry delta directly to points in the
LiDAR's own frame. The static livox_frame -> base_link mounting offset
(5cm forward, no rotation per your config) is small and fixed, so it
mostly cancels out of the relative (capture_time -> reference_time)
correction. If you need higher fidelity, transform points into base_link
using the static extrinsic before interpolating, and back afterward -- for
a 5cm/no-rotation offset the residual error from skipping this is well
under LiDAR range noise.
"""

import bisect
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

# Maps PointField datatype codes -> numpy dtype strings.
_PF_TO_NP = {
    PointField.INT8: 'i1', PointField.UINT8: 'u1',
    PointField.INT16: 'i2', PointField.UINT16: 'u2',
    PointField.INT32: 'i4', PointField.UINT32: 'u4',
    PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8',
}


def field_view(buffer, msg: PointCloud2, field_name: str):
    """
    Zero-copy strided view of ONE PointCloud2 field across all points.

    Deliberately does NOT build one combined structured/record dtype over
    the whole point layout. Livox's PointXYZRTLT format packs a float64
    'timestamp' at byte offset 18, which is not 8-byte aligned -- and
    numpy silently falls back to a slow per-element loop for copy/assign
    operations on structured dtypes with a misaligned field, for the
    WHOLE record, not just that field. That was the actual source of the
    ~80ms/frame cost measured, not the transform math.

    A standalone view of a single scalar dtype (what this returns) goes
    through numpy's ordinary strided-array fast path instead, regardless
    of byte alignment -- the same machinery used for e.g. array[::3].

    `buffer` must support the Python buffer protocol: pass msg.data
    (bytes) for a read-only view, or a bytearray for a writable one.
    """
    f = next(fld for fld in msg.fields if fld.name == field_name)
    endian = '>' if msg.is_bigendian else '<'
    dtype = np.dtype(endian + _PF_TO_NP[f.datatype])
    n = len(buffer) // msg.point_step
    return np.ndarray(shape=(n,), dtype=dtype, buffer=buffer,
                       offset=f.offset, strides=(msg.point_step,))


def quat_to_rotmat(q):
    """q = [x, y, z, w] -> 3x3 rotation matrix."""
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    return np.array([
        [1 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1 - (xx + yy)],
    ])


def quat_slerp(q0, q1, t):
    """Spherical linear interpolation between two quaternions [x,y,z,w]."""
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    dot = np.dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:  # nearly identical -> linear is fine and avoids /0
        result = q0 + t * (q1 - q0)
        return result / np.linalg.norm(result)
    theta0 = np.arccos(dot)
    theta = theta0 * t
    q2 = q1 - q0 * dot
    q2 = q2 / np.linalg.norm(q2)
    return q0 * np.cos(theta) + q2 * np.sin(theta)


class PoseBuffer:
    """Time-sorted buffer of (t_sec, position[3], quat_xyzw[4]) for interpolation."""

    def __init__(self, max_age_sec=2.0):
        self.max_age = max_age_sec
        self._t = []
        self._pos = []
        self._quat = []

    def push(self, t, pos, quat):
        self._t.append(t)
        self._pos.append(pos)
        self._quat.append(quat)
        cutoff = t - self.max_age
        while self._t and self._t[0] < cutoff:
            self._t.pop(0)
            self._pos.pop(0)
            self._quat.pop(0)

    def interpolate(self, t_query):
        """Returns (pos, quat) at t_query, clamped to buffer range if outside it."""
        if not self._t:
            return None
        if t_query <= self._t[0]:
            return self._pos[0], self._quat[0]
        if t_query >= self._t[-1]:
            return self._pos[-1], self._quat[-1]
        i = bisect.bisect_left(self._t, t_query)
        t0, t1 = self._t[i - 1], self._t[i]
        alpha = (t_query - t0) / (t1 - t0) if t1 > t0 else 0.0
        pos = self._pos[i - 1] + alpha * (self._pos[i] - self._pos[i - 1])
        quat = quat_slerp(self._quat[i - 1], self._quat[i], alpha)
        return pos, quat

    def is_empty(self):
        return len(self._t) == 0


class LidarDeskewNode(Node):
    def __init__(self):
        super().__init__('lidar_deskew_node')

        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/livox/lidar/deskewed')
        self.declare_parameter('odom_topic', '/utlidar/robot_odom_restamped')
        self.declare_parameter('pose_buffer_seconds', 2.0)
        self.declare_parameter('reference', 'end')  # 'end' or 'start' of scan
        self.declare_parameter('num_time_bins', 20)
        self.declare_parameter('min_valid_timestamp_spread', 1e-6)  # sec
        self.declare_parameter('log_timing', True)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.reference = self.get_parameter('reference').value
        self.num_bins = int(self.get_parameter('num_time_bins').value)
        self.min_spread = self.get_parameter('min_valid_timestamp_spread').value
        self.log_timing = self.get_parameter('log_timing').value

        self.pose_buf = PoseBuffer(self.get_parameter('pose_buffer_seconds').value)

        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_cb, qos_profile_sensor_data)
        self.cloud_sub = self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_cb, qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, self.output_topic, 10)

        self._warned_no_ts = False
        self._warned_bad_ts = False
        self._warned_no_odom = False

        self.get_logger().info(
            f'Deskewing "{self.input_topic}" -> "{self.output_topic}" '
            f'using odom "{self.odom_topic}" ({self.num_bins} time bins) '
            f'[deskew_node build: fieldview-v2]')

    def odom_cb(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose_buf.push(t, np.array([p.x, p.y, p.z]),
                            np.array([q.x, q.y, q.z, q.w]))

    def cloud_cb(self, msg: PointCloud2):
        t_entry = time.perf_counter()
        field_names = [f.name for f in msg.fields]

        if 'timestamp' not in field_names:
            if not self._warned_no_ts:
                self.get_logger().warn(
                    "Input cloud has no 'timestamp' field -- republishing "
                    "unmodified (expected in Gazebo sim; unexpected on hardware).")
                self._warned_no_ts = True
            self.cloud_pub.publish(msg)
            return

        if self.pose_buf.is_empty():
            if not self._warned_no_odom:
                self.get_logger().warn(
                    f'No odometry received yet on "{self.odom_topic}" -- '
                    'republishing unmodified.')
                self._warned_no_odom = True
            self.cloud_pub.publish(msg)
            return

        n_points = len(msg.data) // msg.point_step
        if n_points == 0:
            self.cloud_pub.publish(msg)
            return

        t_read = time.perf_counter()
        x_in = field_view(msg.data, msg, 'x')
        y_in = field_view(msg.data, msg, 'y')
        z_in = field_view(msg.data, msg, 'z')
        ts_in = field_view(msg.data, msg, 'timestamp')
        timestamps = ts_in.astype(np.float64)

        spread = timestamps.max() - timestamps.min()
        if spread < self.min_spread:
            if not self._warned_bad_ts:
                self.get_logger().warn(
                    f'Per-point timestamp spread is {spread:.2e}s (~0) -- driver '
                    'is not providing real per-point timestamps (known '
                    'livox_ros_driver2 PointCloud2/xfer_format issue). Switch to '
                    'the CustomMsg topic + offset_time, or update the driver. '
                    'Republishing unmodified.')
                self._warned_bad_ts = True
            self.cloud_pub.publish(msg)
            return

        xyz = np.stack([x_in, y_in, z_in], axis=-1).astype(np.float64)

        ref_time = timestamps[-1] if self.reference == 'end' else timestamps[0]
        ref_pose = self.pose_buf.interpolate(ref_time)
        if ref_pose is None:
            self.cloud_pub.publish(msg)
            return
        ref_pos, ref_quat = ref_pose
        R_ref_inv = quat_to_rotmat(ref_quat).T  # rotation matrices are orthonormal

        # Bin points by capture time so we only interpolate + matrix-multiply
        # num_bins times instead of once per point (matters for ~20k pt clouds
        # on the Orin). Increase num_time_bins for more accuracy, at some cost.
        t_min, t_max = timestamps.min(), timestamps.max()
        bin_width = (t_max - t_min) / self.num_bins if t_max > t_min else 1.0
        bin_idx = np.clip(
            ((timestamps - t_min) / bin_width).astype(int), 0, self.num_bins - 1)

        corrected = np.empty_like(xyz)
        for b in range(self.num_bins):
            mask = bin_idx == b
            if not np.any(mask):
                continue
            t_bin = t_min + (b + 0.5) * bin_width
            pose = self.pose_buf.interpolate(t_bin)
            if pose is None:
                corrected[mask] = xyz[mask]
                continue
            pos_i, quat_i = pose
            R_i = quat_to_rotmat(quat_i)
            p_world = xyz[mask] @ R_i.T + pos_i          # capture-time pose -> world
            corrected[mask] = (p_world - ref_pos) @ R_ref_inv.T  # world -> reference pose

        t_transform = time.perf_counter()

        # Single raw memcpy of the whole message, then overwrite just x/y/z
        # in place via writable strided views -- no structured-dtype field
        # copy involved anywhere in this path.
        out_bytes = bytearray(msg.data)
        x_out = field_view(out_bytes, msg, 'x')
        y_out = field_view(out_bytes, msg, 'y')
        z_out = field_view(out_bytes, msg, 'z')
        x_out[:] = corrected[:, 0].astype(x_out.dtype)
        y_out[:] = corrected[:, 1].astype(y_out.dtype)
        z_out[:] = corrected[:, 2].astype(z_out.dtype)

        out_msg = PointCloud2()
        out_msg.header = msg.header
        out_msg.height = msg.height
        out_msg.width = msg.width
        out_msg.fields = msg.fields
        out_msg.is_bigendian = msg.is_bigendian
        out_msg.point_step = msg.point_step
        out_msg.row_step = msg.row_step
        out_msg.is_dense = msg.is_dense
        out_msg.data = bytes(out_bytes)
        t_write = time.perf_counter()

        self.cloud_pub.publish(out_msg)
        t_publish = time.perf_counter()

        if self.log_timing:
            self.get_logger().info(
                f'[deskew timing] n_pts={n_points} '
                f'read={1000*(t_read - t_entry):.1f}ms '
                f'transform={1000*(t_transform - t_read):.1f}ms '
                f'write={1000*(t_write - t_transform):.1f}ms '
                f'publish={1000*(t_publish - t_write):.1f}ms '
                f'total={1000*(t_publish - t_entry):.1f}ms',
                throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = LidarDeskewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()