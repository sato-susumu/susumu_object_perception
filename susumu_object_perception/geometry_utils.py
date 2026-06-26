"""Lightweight geometry helpers (math-only, no numpy/ROS dependency).

IMU 系の小さいノード (step_detector_node, fall_detector_node) で重複していた
quaternion -> Euler 変換などをまとめる軽量モジュール。 numpy / ROS に依存しないので
import コストが小さい。 3D 数学全般 (回転行列・投影行列) は重い omni_projection.py を
使う。
"""

import math


def quat_to_roll_pitch(q):
    """quaternion (x, y, z, w) -> (roll, pitch) in radians.

    geometry_msgs/Quaternion 互換オブジェクト (`.x .y .z .w` 属性を持つもの) を受ける。
    sinp が 1 を超えるジンバルロック近傍は `±pi/2` を返す。
    yaw は外部 (地磁気・SLAM) で別途扱う前提で省略。
    """
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    return roll, pitch
