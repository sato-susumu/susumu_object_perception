#!/usr/bin/env python3
"""AprilTag を使った全天球カメラ + 3D LiDAR の外部キャリブレーションノード。

docs/omni_lidar_camera.md「AprilTag 既知ターゲット方式」の実装。`apriltag_ros` はピンホール
rectified 画像前提で全天球(equirectangular)に直接使えないため、本ノードは OpenCV の
`cv2.aruco`(AprilTag 36h11) で検出する自前パイプラインにする。独自 `.msg` は作らず、入力は
標準型(`sensor_msgs/Image`, `sensor_msgs/PointCloud2`)、出力は `direct_visual_lidar_calibration`
互換の `calib.json`(`results.T_lidar_camera` = [x,y,z,qx,qy,qz,qw])にする。これで既存の
`omni_sensor_tf_node.py` / `scripts/direct_calib_to_tf.py` がそのまま TF 置換に使える。

処理フロー:
  1. 全天球 equirect 画像と LiDAR 点群を購読し、安定後に N フレーム集める。
  2. カメラ側: タグ既知方位ごとに透視ビュー展開 → `cv2.aruco` でタグ検出 → 仮想ピンホール
     intrinsic で `cv2.solvePnP` → タグ中心のカメラ座標(omni_camera_link)を得る。
  3. LiDAR側: 同方位の点群を方位角で切り出し → 平面 RANSAC → タグ板中心を LiDAR 座標で得る。
  4. タグごとに (camera中心, lidar中心) の 3D 点対応を作り、Umeyama(剛体)で T_lidar_camera 推定。
  5. calib.json を出力。

T_lidar_camera は p_lidar = T_lidar_camera * p_camera の向き(vlcal と同一規約)。
"""

import json
import math
import os
from collections import defaultdict

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

import cv2
from cv_bridge import CvBridge

from susumu_object_perception.omni_projection import (
    equirect_uv, perspective_directions)


def equirect_to_perspective_with_dirs(pano, yaw, pitch, fov, out_w, out_h,
                                      projection_model='webots_cylindrical'):
    """正距円筒画像から透視ビューを切り出し、各画素のカメラ座標レイ方向も返す。

    traffic_light_detector_node.equirect_to_perspective と同じ投影式。返す ``dirs`` は
    形 (out_h, out_w, 3) で、ビュー画素 (col,row) に対応するカメラ座標(x=前方,y=左,z=上)の
    単位方向。タグ検出後に画素→3D方向の逆算に使う。
    """
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    forward = np.array([cp * cy, cp * sy, sp], dtype=np.float32)
    dirs = perspective_directions(forward, out_w, out_h, fov)
    h, w = pano.shape[:2]
    map_x, map_y, _ = equirect_uv(dirs, w, h, projection_model)
    view = cv2.remap(pano, map_x, map_y, interpolation=cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_WRAP)
    return view, dirs


def virtual_intrinsics(out_w, out_h, fov):
    """透視ビューの仮想ピンホール内部行列 K を返す（perspective_directions と整合）。

    perspective_directions は forward + right*(x*tan(fov/2)*aspect) + up*(-y*tan(fov/2))、
    x,y は [-1,1] の正規化座標。ビュー中心が主点、焦点距離はこの正規化と一致させる。
    """
    aspect = out_w / float(out_h)
    tan_half = math.tan(fov / 2.0)
    fx = (out_w / 2.0) / (tan_half * aspect)
    fy = (out_h / 2.0) / tan_half
    cx = (out_w - 1) / 2.0
    cy = (out_h - 1) / 2.0
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)


def view_to_camera_rotation(yaw, pitch):
    """透視ビュー座標(右=+x_v, 下=+y_v, 前=+z_v) → カメラ座標(x前,y左,z上) の回転。

    perspective_directions の forward/right/up と同じ基底で組む。solvePnP が返す姿勢は
    「ビューのカメラ座標(光軸+z)」基準なので、これでカメラ(omni_camera_link)座標へ移す。
    """
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    forward = np.array([cp * cy, cp * sy, sp], dtype=np.float64)
    world_up = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(forward, world_up))) > 0.95:
        world_up = np.array([0.0, 1.0, 0.0])
    right = np.cross(forward, world_up)
    right /= max(np.linalg.norm(right), 1e-9)
    up = np.cross(right, forward)
    up /= max(np.linalg.norm(up), 1e-9)
    # ビュー: 右=+x_v, 下=+y_v(=-up), 前(光軸)=+z_v(=forward)。
    # 列ベクトルに各ビュー軸のカメラ座標表現を入れる。
    return np.column_stack([right, -up, forward])


def umeyama(src, dst):
    """src(Nx3) を dst(Nx3) に合わせる剛体変換 R,t を返す（p_dst = R*p_src + t）。"""
    src = np.asarray(src, dtype=np.float64)
    dst = np.asarray(dst, dtype=np.float64)
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    sc = src - mu_s
    dc = dst - mu_d
    H = sc.T @ dc
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T
    t = mu_d - R @ mu_s
    return R, t


def rot_to_quat(R):
    """回転行列 → クォータニオン [qx,qy,qz,qw]。"""
    m = R
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    return q / np.linalg.norm(q)


class AprilTagExtrinsicCalibNode(Node):

    def __init__(self):
        super().__init__('apriltag_extrinsic_calib')
        self.declare_parameter('input_image',
                               '/omni_camera/image_raw/image_color')
        self.declare_parameter('input_cloud', '/lidar/points')
        self.declare_parameter('projection_model', 'webots_cylindrical')
        # タグ ID → 既知方位[deg]（calibration.wbt の 4 パネル配置に対応）。
        # FRONT(+x)=id0=0°, LEFT(+y)=id1=90°, BACK(-x)=id2=180°, RIGHT(-y)=id3=270°。
        self.declare_parameter('tag_ids', [0, 1, 2, 3])
        self.declare_parameter('tag_yaws_deg', [0.0, 90.0, 180.0, 270.0])
        # タグの物理エッジ長[m]。テクスチャのタグ占有率 0.6667 × 板 1 辺 1.0m。
        self.declare_parameter('tag_size', 0.6667)
        # 透視ビューの FOV[deg] と解像度。
        self.declare_parameter('view_fov_deg', 70.0)
        self.declare_parameter('view_width', 800)
        self.declare_parameter('view_height', 800)
        # 集めるフレーム数（複数フレームで中心を平均し安定化）。
        self.declare_parameter('num_frames', 20)
        self.declare_parameter('startup_sec', 8.0)
        # LiDAR 板抽出: 方位±half[deg]、距離[min,max]m、平面 RANSAC しきい[m]。
        self.declare_parameter('lidar_az_halfwidth_deg', 12.0)
        self.declare_parameter('lidar_range_min', 1.0)
        self.declare_parameter('lidar_range_max', 3.5)
        self.declare_parameter('plane_ransac_thresh', 0.03)
        self.declare_parameter('lidar_z_min', -0.3)
        self.declare_parameter('lidar_z_max', 1.2)
        # 板厚補正[m]。LiDAR は板の手前面で反射するため平面フィット中心は板物理中心より
        # 板厚/2 だけ LiDAR 側に寄る。重心を「LiDAR→板の視線方向」に board_thickness/2 押し戻して
        # 物理中心（カメラ PnP が見るタグ面中心と同じ）に合わせ、x 方向の系統オフセットを消す。
        # calibration.wbt のパネル Box 厚 0.04m → 補正 0.02m。0.0 で無効。
        self.declare_parameter('board_thickness', 0.04)
        # 【LiDAR 上向き FOV 補正】LiDAR が板の下半分にしか点を返さない場合、 重心の
        # z 座標は板物理中心より下に偏る。 z 座標を「点群の z 範囲中央 (max+min)/2」
        # に置き換えると、 点が下半分に偏っていても z は板中央近くになる。
        # True で有効。 docs/tasks/extrinsic_calibration.md の「並進絶対誤差 24mm 」
        # 対策 (LiDAR が板の下半分にしか点を返さず重心が下に偏る) の候補。
        self.declare_parameter('lidar_z_use_range_mid', False)
        self.declare_parameter('output_json', os.path.expanduser(
            '~/ros2_ws/src/susumu_object_perception/outputs/extrinsic_calibration/calib.json'))
        # 既知初期 TF（lidar_link -> omni_camera_link、検証時の参照）。
        self.declare_parameter('ref_translation', [0.0, 0.0, 0.55])

        self.input_image = self.get_parameter('input_image').value
        self.input_cloud = self.get_parameter('input_cloud').value
        self.projection_model = self.get_parameter('projection_model').value
        self.tag_ids = [int(v) for v in self.get_parameter('tag_ids').value]
        self.tag_yaws = [math.radians(float(v))
                         for v in self.get_parameter('tag_yaws_deg').value]
        self.tag_size = float(self.get_parameter('tag_size').value)
        self.view_fov = math.radians(
            float(self.get_parameter('view_fov_deg').value))
        self.view_w = int(self.get_parameter('view_width').value)
        self.view_h = int(self.get_parameter('view_height').value)
        self.num_frames = int(self.get_parameter('num_frames').value)
        self.startup_sec = float(self.get_parameter('startup_sec').value)
        self.az_half = math.radians(
            float(self.get_parameter('lidar_az_halfwidth_deg').value))
        self.range_min = float(self.get_parameter('lidar_range_min').value)
        self.range_max = float(self.get_parameter('lidar_range_max').value)
        self.plane_thresh = float(self.get_parameter('plane_ransac_thresh').value)
        self.z_min = float(self.get_parameter('lidar_z_min').value)
        self.z_max = float(self.get_parameter('lidar_z_max').value)
        self.board_thickness = float(self.get_parameter('board_thickness').value)
        self.lidar_z_use_range_mid = bool(
            self.get_parameter('lidar_z_use_range_mid').value)
        self.output_json = self.get_parameter('output_json').value
        self.ref_translation = [float(v)
                                for v in self.get_parameter('ref_translation').value]

        self.id_to_yaw = {tid: self.tag_yaws[i]
                          for i, tid in enumerate(self.tag_ids)
                          if i < len(self.tag_yaws)}

        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36h11)
        self.detector = cv2.aruco.ArucoDetector(
            self.dictionary, cv2.aruco.DetectorParameters())
        self.K = virtual_intrinsics(self.view_w, self.view_h, self.view_fov)
        self.dist = np.zeros(5, dtype=np.float64)

        # タグ ID → 観測した中心の蓄積（cam 座標 / lidar 座標）。
        self.cam_centers = defaultdict(list)
        self.lidar_centers = defaultdict(list)

        self.latest_image = None
        self.latest_cloud = None
        self._frames_done = 0
        self._finished = False

        self.create_subscription(Image, self.input_image,
                                 self._on_image, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.input_cloud,
                                 self._on_cloud, qos_profile_sensor_data)
        self.create_timer(self.startup_sec, self._start_once)
        self._collect_timer = None
        self.get_logger().info(
            'apriltag_extrinsic_calib started '
            f'(tags={self.tag_ids} size={self.tag_size:.3f}m '
            f'frames={self.num_frames} -> {self.output_json})')

    def _on_image(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'omni image decode failed: {exc}')

    def _on_cloud(self, msg):
        self.latest_cloud = msg

    def _start_once(self):
        self._collect_timer = self.create_timer(0.5, self._collect)

    # ---- カメラ側: タグ検出 + PnP ---------------------------------------

    def _detect_tags_in_camera(self, pano):
        """全方位の透視ビューでタグを検出し、{id: cam中心(3,)} を返す。"""
        out = {}
        for tid, yaw in self.id_to_yaw.items():
            view, _ = equirect_to_perspective_with_dirs(
                pano, yaw, 0.0, self.view_fov, self.view_w, self.view_h,
                self.projection_model)
            corners, ids, _ = self.detector.detectMarkers(view)
            if ids is None:
                continue
            ids = ids.ravel()
            if tid not in ids:
                continue
            idx = int(np.where(ids == tid)[0][0])
            img_pts = corners[idx].reshape(4, 2).astype(np.float64)
            # タグ角の 3D モデル点（タグ中心原点、タグ面 z=0、反時計回り）。
            h = self.tag_size / 2.0
            obj_pts = np.array([[-h, h, 0], [h, h, 0],
                                [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts, self.K, self.dist,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            # tvec はビューのカメラ座標(光軸 +z)系でのタグ中心。
            center_view = tvec.reshape(3)
            R_cam_view = view_to_camera_rotation(yaw, 0.0)
            center_cam = R_cam_view @ center_view
            out[tid] = center_cam
        return out

    # ---- LiDAR 側: 方位切り出し + 平面フィット ---------------------------

    def _detect_boards_in_lidar(self, cloud_msg):
        """各タグ方位の点群から板中心(lidar座標)を抽出し {id: lidar中心(3,)} を返す。"""
        pts = pc2.read_points_numpy(
            cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.size == 0:
            return {}
        pts = pts.reshape(-1, 3).astype(np.float64)
        rng = np.linalg.norm(pts[:, :2], axis=1)
        az = np.arctan2(pts[:, 1], pts[:, 0])
        m_common = (rng >= self.range_min) & (rng <= self.range_max) & \
                   (pts[:, 2] >= self.z_min) & (pts[:, 2] <= self.z_max)
        out = {}
        for tid, yaw in self.id_to_yaw.items():
            daz = np.arctan2(np.sin(az - yaw), np.cos(az - yaw))
            sel = m_common & (np.abs(daz) <= self.az_half)
            seg = pts[sel]
            if seg.shape[0] < 20:
                continue
            center = self._plane_center(seg)
            if center is None:
                continue
            # 板厚補正: LiDAR は板手前面で反射するので重心は物理中心より LiDAR 側に寄る。
            # LiDAR 原点 → 重心の視線方向（板へ向かう向き）に board_thickness/2 押し戻す。
            if self.board_thickness > 0.0:
                los = center / max(np.linalg.norm(center), 1e-9)
                center = center + los * (self.board_thickness / 2.0)
            out[tid] = center
        return out

    def _plane_center(self, seg):
        """点群塊に平面 RANSAC をかけ、インライアの重心を返す。"""
        best_inliers = None
        best_count = 0
        n = seg.shape[0]
        rng = np.random.default_rng(0)
        for _ in range(100):
            idx = rng.choice(n, size=3, replace=False)
            p0, p1, p2 = seg[idx]
            normal = np.cross(p1 - p0, p2 - p0)
            nn = np.linalg.norm(normal)
            if nn < 1e-6:
                continue
            normal = normal / nn
            dist = np.abs((seg - p0) @ normal)
            inliers = dist < self.plane_thresh
            count = int(inliers.sum())
            if count > best_count:
                best_count = count
                best_inliers = inliers
        if best_inliers is None or best_count < 15:
            return None
        inliers = seg[best_inliers]
        center = inliers.mean(axis=0)
        if self.lidar_z_use_range_mid:
            # LiDAR が板下半分しか取れない条件で z 重心が下に偏るのを矯正。
            # x, y は重心のまま、 z だけ点群 z 範囲中央 (max+min)/2 に置換。
            z_mid = (float(inliers[:, 2].max()) +
                     float(inliers[:, 2].min())) * 0.5
            center = np.array([center[0], center[1], z_mid])
        return center

    # ---- 収集 + 推定 -----------------------------------------------------

    def _collect(self):
        if self._finished:
            return
        if self.latest_image is None or self.latest_cloud is None:
            return
        cam = self._detect_tags_in_camera(self.latest_image)
        lid = self._detect_boards_in_lidar(self.latest_cloud)
        common = set(cam) & set(lid)
        for tid in common:
            self.cam_centers[tid].append(cam[tid])
            self.lidar_centers[tid].append(lid[tid])
        self._frames_done += 1
        self.get_logger().info(
            f'frame {self._frames_done}/{self.num_frames}: '
            f'cam tags={sorted(cam)} lidar boards={sorted(lid)} '
            f'matched={sorted(common)}')
        if self._frames_done >= self.num_frames:
            self._finish()

    def _finish(self):
        self._finished = True
        if self._collect_timer is not None:
            self._collect_timer.cancel()
        cam_pts = []
        lid_pts = []
        used = []
        for tid in sorted(self.cam_centers):
            if tid not in self.lidar_centers:
                continue
            c = np.mean(self.cam_centers[tid], axis=0)
            l = np.mean(self.lidar_centers[tid], axis=0)
            cam_pts.append(c)
            lid_pts.append(l)
            used.append(tid)
        if len(cam_pts) < 3:
            self.get_logger().error(
                f'not enough tag correspondences ({len(cam_pts)} < 3); '
                'cannot estimate extrinsics. タグ/板検出を見直す')
            return
        cam_pts = np.array(cam_pts)
        lid_pts = np.array(lid_pts)
        # p_lidar = R * p_camera + t を解く（Umeyama, src=cam, dst=lidar）。
        R, t = umeyama(cam_pts, lid_pts)
        resid = lid_pts - (cam_pts @ R.T + t)
        rms = float(np.sqrt(np.mean(np.sum(resid ** 2, axis=1))))
        quat = rot_to_quat(R)
        self._write_json(t, quat, used, rms)
        ref = np.array(self.ref_translation)
        derr = float(np.linalg.norm(t - ref))
        self.get_logger().info(
            f'=== AprilTag extrinsic calib done ===\n'
            f'  used tags: {used}\n'
            f'  T_lidar_camera translation: '
            f'[{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}] m\n'
            f'  T_lidar_camera quat(xyzw): '
            f'[{quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f}]\n'
            f'  correspondence RMS: {rms:.4f} m\n'
            f'  translation error vs ref {self.ref_translation}: {derr:.4f} m\n'
            f'  wrote: {self.output_json}')

    def _write_json(self, t, quat, used, rms):
        os.makedirs(os.path.dirname(self.output_json), exist_ok=True)
        data = {
            'meta': {
                'method': 'apriltag_36h11_omni_lidar',
                'data_path': 'webots_calibration_apriltag',
                'camera_model': 'equirectangular',
            },
            'results': {
                # p_lidar = T_lidar_camera * p_camera。[x,y,z,qx,qy,qz,qw]。
                'T_lidar_camera': [float(t[0]), float(t[1]), float(t[2]),
                                   float(quat[0]), float(quat[1]),
                                   float(quat[2]), float(quat[3])],
            },
            'apriltag_calib': {
                'used_tag_ids': [int(v) for v in used],
                'correspondence_rms_m': rms,
                'tag_size_m': self.tag_size,
            },
        }
        with open(self.output_json, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagExtrinsicCalibNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
