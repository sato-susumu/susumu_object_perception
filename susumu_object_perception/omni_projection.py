"""Shared projection helpers for the Webots omnidirectional camera."""

import math

import numpy as np


WEBOTS_CYLINDRICAL_ROT = np.array([
    [0.0, 0.0, -1.0],
    [0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0],
], dtype=np.float32)


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3, dtype=np.float32)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - yy - zz, xy - wz, xz + wy],
        [xy + wz, 1.0 - xx - zz, yz - wx],
        [xz - wy, yz + wx, 1.0 - xx - yy],
    ], dtype=np.float32)


def euler_xyz_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float32)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float32)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float32)
    return rz @ ry @ rx


def equirect_uv(points, width, height, projection_model='webots_cylindrical',
                yaw_offset=0.0, pitch_offset=0.0):
    """Project camera-frame directions/points to an equirectangular image.

    Returns float32 ``(u, v, valid)`` arrays shaped like ``points[..., 0]``.
    ``projection_model='webots_cylindrical'`` follows the Webots cylindrical
    camera shader convention used by the simulated omnidirectional camera.
    """
    pts = np.asarray(points, dtype=np.float32)
    if projection_model == 'webots_cylindrical':
        pts = pts @ WEBOTS_CYLINDRICAL_ROT.T
        x = pts[..., 0]
        y = pts[..., 1]
        z = pts[..., 2]
        r = np.linalg.norm(pts, axis=-1)
        valid = r > 1e-6
        yaw = np.arctan2(y, x) + yaw_offset
        z_unit = np.clip(z / np.maximum(r, 1e-6), -1.0, 1.0)
        v_angle = np.arccos(z_unit) - math.pi / 2.0
        u = (0.5 - yaw / (2.0 * math.pi)) * width
        v = (0.5 + (v_angle + pitch_offset) / math.pi) * height
        valid &= (v >= 0.0) & (v < height)
        return (u % width).astype(np.float32), v.astype(np.float32), valid

    x = pts[..., 0]
    y = pts[..., 1]
    z = pts[..., 2]
    r = np.linalg.norm(pts, axis=-1)
    valid = r > 1e-6
    yaw = np.arctan2(-y, x) + yaw_offset
    pitch = np.arcsin(np.clip(z / np.maximum(r, 1e-6), -1.0, 1.0))
    pitch = np.clip(pitch + pitch_offset, -math.pi / 2.0, math.pi / 2.0)
    u = ((yaw + math.pi) / (2.0 * math.pi) * width) % width
    v = (math.pi / 2.0 + pitch) / math.pi * height
    valid &= (v >= 0.0) & (v < height)
    return u.astype(np.float32), v.astype(np.float32), valid


def perspective_directions(direction, out_w, out_h, fov_rad):
    """Build a perspective view ray grid centered on ``direction``."""
    direction = np.asarray(direction, dtype=np.float32)
    norm = np.linalg.norm(direction)
    if norm < 1e-6:
        return None
    forward = direction / norm
    world_up = np.array([0.0, 0.0, 1.0], dtype=np.float32)
    if abs(float(np.dot(forward, world_up))) > 0.95:
        world_up = np.array([0.0, 1.0, 0.0], dtype=np.float32)
    right = np.cross(forward, world_up)
    right /= max(np.linalg.norm(right), 1e-6)
    up = np.cross(right, forward)
    up /= max(np.linalg.norm(up), 1e-6)

    xs = np.linspace(-1.0, 1.0, out_w, dtype=np.float32)
    ys = np.linspace(-1.0, 1.0, out_h, dtype=np.float32)
    xx, yy = np.meshgrid(xs, ys)
    tan_half = math.tan(fov_rad / 2.0)
    aspect = out_w / float(out_h)
    dirs = (forward.reshape(1, 1, 3) +
            right.reshape(1, 1, 3) * (xx[..., None] * tan_half * aspect) +
            up.reshape(1, 1, 3) * (-yy[..., None] * tan_half))
    dirs /= np.maximum(np.linalg.norm(dirs, axis=2, keepdims=True), 1e-6)
    return dirs
