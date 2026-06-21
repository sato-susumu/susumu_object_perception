#!/usr/bin/env python3
"""Record object-classifier debug diagnostics during a recognition run.

The object classifier already publishes DiagnosticArray entries for every YOLO
candidate when object_classifier_debug:=True.  This node turns those live
diagnostics into target-oriented reports so a debug waypoint run can answer:

* Did the target track receive a crop?
* Did YOLO return no candidate, a rejected candidate, or an accepted one?
* Which fine classes were selected near each target?
"""

import argparse
from collections import Counter, defaultdict, deque
import csv
import json
import math
import os
import re
import signal
import time

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from autoware_perception_msgs.msg import DetectedObjects, TrackedObjects
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def _stamp_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _status_values(status):
    return {kv.key: kv.value for kv in status.values}


def _crop_key(values):
    fov = values.get('crop_fov_deg', '')
    yaw = values.get('crop_yaw_offset_deg', '')
    pitch = values.get('crop_pitch_offset_deg', '')
    height_frac = values.get('crop_shape_height_frac', '')
    crop_mode = values.get('crop_mode', '')
    bbox_margin = values.get('crop_shape_bbox_margin_deg', '')
    bbox_fov = values.get('crop_shape_bbox_projected_fov_deg', '')
    if (yaw != '' or pitch != '' or height_frac != '' or
            crop_mode != '' or bbox_margin != '' or bbox_fov != ''):
        key = f'{fov}/yaw{yaw}/pitch{pitch}'
        if height_frac != '':
            key += f'/h{height_frac}'
        if crop_mode != '':
            key += f'/{crop_mode}'
        if bbox_margin != '':
            key += f'/bboxm{bbox_margin}'
        if bbox_fov != '':
            key += f'/pfov{bbox_fov}'
        return key
    return fov


def _as_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _counter_to_dict(counter):
    return {str(k): int(v) for k, v in counter.items()}


def _top(counter, limit=5):
    return ', '.join(f'{k}:{v}' for k, v in counter.most_common(limit))


def _label_from_object(obj):
    if not obj.classification:
        return -1
    return int(obj.classification[0].label)


def _shape_dict(obj):
    d = obj.shape.dimensions
    return {
        'x': float(d.x),
        'y': float(d.y),
        'z': float(d.z),
    }


def _shape_txt(shape):
    if not shape:
        return ''
    return (
        f"{shape.get('x', 0.0):.2f}x"
        f"{shape.get('y', 0.0):.2f}x"
        f"{shape.get('z', 0.0):.2f}")


def _distance_bin(d):
    if d < 0.5:
        return '<0.5'
    if d < 1.0:
        return '0.5-1.0'
    if d < 1.5:
        return '1.0-1.5'
    if d < 2.0:
        return '1.5-2.0'
    return '>=2.0'


class RecognitionDebugRecorder(Node):

    def __init__(self, args):
        super().__init__('recognition_debug_recorder')
        self.args = args
        self.started_wall = time.monotonic()
        self.last_write_wall = 0.0
        self.stop_requested = False
        self.detected_stage_topics = self._parse_stage_topics(
            args.detected_stage_topic)
        self.tracked_stage_topics = self._parse_stage_topics(
            args.tracked_stage_topic)
        self.stage_names = [
            name for name, _topic in
            (self.detected_stage_topics + self.tracked_stage_topics)
        ]
        self.targets = self._load_targets(args.target_waypoints)
        self.target_by_waypoint = {
            int(t['waypoint_index']): t['eid']
            for t in self.targets
            if t.get('waypoint_index') is not None
        }
        self.target_stats = {
            t['eid']: self._new_target_stats(t) for t in self.targets
        }
        self.global_stats = {
            'debug_messages': 0,
            'candidate_statuses': 0,
            'fine_class_messages': 0,
            'tracked_messages': 0,
            'waypoint_status_messages': 0,
            'unmatched_debug_statuses': 0,
            'unmatched_fine_class_statuses': 0,
            'tf_failures': 0,
            'stage_messages': Counter(),
        }
        self.object_positions = {}
        self.fine_classes = {}
        self.waypoint_statuses = deque(maxlen=args.keep_events)
        self.active_waypoint = None
        self.active_target_eid = None
        if args.active_waypoint_fallback and len(self.targets) == 1:
            self.active_target_eid = self.targets[0]['eid']
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            DiagnosticArray, args.debug_topic, self.on_debug, 10)
        self.create_subscription(
            DiagnosticArray, args.fine_class_topic, self.on_fine_classes, 10)
        self.create_subscription(
            TrackedObjects, args.tracked_topic, self.on_tracked_objects,
            qos_profile_sensor_data)
        self.create_subscription(
            DiagnosticArray, args.tracker_debug_topic,
            self.on_tracker_debug, 10)
        for stage_name, topic in self.detected_stage_topics:
            self.create_subscription(
                DetectedObjects, topic,
                self._make_detected_stage_cb(stage_name),
                qos_profile_sensor_data)
        for stage_name, topic in self.tracked_stage_topics:
            self.create_subscription(
                TrackedObjects, topic,
                self._make_tracked_stage_cb(stage_name),
                qos_profile_sensor_data)
        self.create_subscription(
            String, args.waypoint_status_topic, self.on_waypoint_status, 10)
        self.timer = self.create_timer(
            max(0.2, args.write_period_sec), self.on_timer)
        self.get_logger().info(
            f'recording recognition debug for {len(self.targets)} targets '
            f'to {args.out_prefix}.md/.json/.csv')

    @staticmethod
    def _parse_stage_topics(items):
        out = []
        seen = set()
        for item in items or []:
            if not item:
                continue
            if ':' not in item:
                raise RuntimeError(
                    f'stage topic must be NAME:/topic, got: {item}')
            name, topic = item.split(':', 1)
            name = name.strip()
            topic = topic.strip()
            if not name or not topic:
                raise RuntimeError(
                    f'stage topic must be NAME:/topic, got: {item}')
            if name in seen:
                continue
            seen.add(name)
            out.append((name, topic))
        return out

    @staticmethod
    def _load_targets(path):
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f)
        targets = []
        for row in data.get('targets', []):
            if 'eid' not in row or 'map_x' not in row or 'map_y' not in row:
                continue
            targets.append({
                'eid': str(row['eid']),
                'wbt_type': str(row.get('wbt_type', '')),
                'name': str(row.get('name', '')),
                'map_x': float(row['map_x']),
                'map_y': float(row['map_y']),
                'waypoint_index': row.get('waypoint_index'),
            })
        if not targets:
            raise RuntimeError(f'no targets in {path}')
        return targets

    def _new_target_stats(self, target):
        return {
            'target': dict(target),
            'debug_status_count': 0,
            'candidate_status_count': 0,
            'no_yolo_detection_count': 0,
            'accepted_count': 0,
            'rejected_count': 0,
            'selected_count': 0,
            'fine_class_count': 0,
            'tracked_observation_count': 0,
            'nearest_tracked_distance_m': None,
            'object_ids': set(),
            'candidate_classes': Counter(),
            'accepted_classes': Counter(),
            'selected_classes': Counter(),
            'fine_classes': Counter(),
            'reject_reasons': Counter(),
            'fovs': Counter(),
            'tracker_debug_status_count': 0,
            'tracker_debug_near_count': 0,
            'tracker_debug_nearest_distance_m': None,
            'tracker_debug_reasons': Counter(),
            'tracker_debug_events': deque(maxlen=self.args.keep_events),
            'object_associations': {},
            'stages': {
                name: {
                    'message_count': 0,
                    'nearest_sample_count': 0,
                    'within_match_count': 0,
                    'nearest_distance_m': None,
                    'nearest_object_id': '',
                    'nearest_label': -1,
                    'nearest_shape': None,
                    'distance_bins': Counter(),
                    'events': deque(maxlen=self.args.keep_events),
                }
                for name in self.stage_names
            },
            'events': deque(maxlen=self.args.keep_events),
        }

    def _new_object_assoc(self, object_id):
        return {
            'object_id': object_id,
            'tracked_count': 0,
            'debug_count': 0,
            'no_yolo_count': 0,
            'accepted_count': 0,
            'rejected_count': 0,
            'selected_count': 0,
            'fine_count': 0,
            'tracker_debug_count': 0,
            'nearest_distance_m': None,
            'last_distance_m': None,
            'last_x': None,
            'last_y': None,
            'last_label': -1,
            'nearest_shape': None,
            'last_shape': None,
            'distance_bins': Counter(),
            'stage_counts': Counter(),
            'candidate_classes': Counter(),
            'accepted_classes': Counter(),
            'selected_classes': Counter(),
            'fine_classes': Counter(),
            'reject_reasons': Counter(),
            'tracker_reasons': Counter(),
            'fovs': Counter(),
            'events': deque(maxlen=self.args.keep_events),
        }

    def _object_assoc(self, target_eid, object_id):
        stats = self.target_stats[target_eid]
        object_id = str(object_id or '')
        return stats['object_associations'].setdefault(
            object_id, self._new_object_assoc(object_id))

    def _update_object_assoc_pose(self, assoc, dist=None, x=None, y=None,
                                  shape=None, label=None, stage=None):
        if dist is not None:
            assoc['last_distance_m'] = float(dist)
            assoc['distance_bins'][_distance_bin(float(dist))] += 1
            prev = assoc['nearest_distance_m']
            if prev is None or dist < prev:
                assoc['nearest_distance_m'] = float(dist)
                if shape is not None:
                    assoc['nearest_shape'] = shape
        if x is not None:
            assoc['last_x'] = float(x)
        if y is not None:
            assoc['last_y'] = float(y)
        if label is not None:
            assoc['last_label'] = int(label)
        if shape is not None:
            assoc['last_shape'] = shape
            if assoc['nearest_shape'] is None:
                assoc['nearest_shape'] = shape
        if stage:
            assoc['stage_counts'][stage] += 1

    @staticmethod
    def _serializable_object_associations(stats):
        rows = []
        for assoc in stats['object_associations'].values():
            rows.append({
                'object_id': assoc['object_id'],
                'tracked_count': int(assoc['tracked_count']),
                'debug_count': int(assoc['debug_count']),
                'no_yolo_count': int(assoc['no_yolo_count']),
                'accepted_count': int(assoc['accepted_count']),
                'rejected_count': int(assoc['rejected_count']),
                'selected_count': int(assoc['selected_count']),
                'fine_count': int(assoc['fine_count']),
                'tracker_debug_count': int(assoc['tracker_debug_count']),
                'nearest_distance_m': assoc['nearest_distance_m'],
                'last_distance_m': assoc['last_distance_m'],
                'last_x': assoc['last_x'],
                'last_y': assoc['last_y'],
                'last_label': int(assoc['last_label']),
                'nearest_shape': assoc['nearest_shape'],
                'last_shape': assoc['last_shape'],
                'distance_bins': _counter_to_dict(assoc['distance_bins']),
                'stage_counts': _counter_to_dict(assoc['stage_counts']),
                'candidate_classes': _counter_to_dict(
                    assoc['candidate_classes']),
                'accepted_classes': _counter_to_dict(
                    assoc['accepted_classes']),
                'selected_classes': _counter_to_dict(
                    assoc['selected_classes']),
                'fine_classes': _counter_to_dict(assoc['fine_classes']),
                'reject_reasons': _counter_to_dict(assoc['reject_reasons']),
                'tracker_reasons': _counter_to_dict(
                    assoc['tracker_reasons']),
                'fovs': _counter_to_dict(assoc['fovs']),
                'events': list(assoc['events']),
            })

        def _rank(row):
            nearest = row['nearest_distance_m']
            nearest = float('inf') if nearest is None else float(nearest)
            evidence = (
                row['tracked_count'] + row['debug_count'] +
                row['fine_count'] + row['tracker_debug_count'] +
                sum(int(v) for v in row['stage_counts'].values()))
            return (nearest, -evidence, row['object_id'])

        return sorted(rows, key=_rank)

    def _nearest_target(self, x, y):
        best = None
        for target in self.targets:
            d = math.hypot(x - target['map_x'], y - target['map_y'])
            if best is None or d < best[0]:
                best = (d, target)
        if best is None or best[0] > self.args.target_match_distance:
            return None, best[0] if best else None
        return best[1], best[0]

    def _target_for_object(self, object_id):
        pos = self.object_positions.get(object_id)
        if pos is not None:
            target, dist = self._nearest_target(pos['x'], pos['y'])
            if target is not None:
                return target['eid'], dist, 'position'
        if self.args.active_waypoint_fallback and self.active_target_eid:
            return self.active_target_eid, None, 'active_waypoint'
        return None, None, 'unmatched'

    def _update_target_distance(self, target_eid, dist):
        if target_eid is None or dist is None:
            return
        stats = self.target_stats[target_eid]
        prev = stats['nearest_tracked_distance_m']
        if prev is None or dist < prev:
            stats['nearest_tracked_distance_m'] = float(dist)

    def _transform_to_map(self, x, y, frame_id):
        if not frame_id or frame_id == self.args.map_frame:
            return x, y
        try:
            tf = self.tf_buffer.lookup_transform(
                self.args.map_frame, frame_id, rclpy.time.Time())
        except TransformException as exc:
            self.global_stats['tf_failures'] += 1
            self.get_logger().warn(
                f'TF {self.args.map_frame}<-{frame_id} unavailable: {exc}',
                throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c = math.cos(yaw)
        s = math.sin(yaw)
        return c * x - s * y + t.x, s * x + c * y + t.y

    def _make_detected_stage_cb(self, stage_name):
        def _cb(msg):
            self.on_detected_stage(stage_name, msg)
        return _cb

    def _make_tracked_stage_cb(self, stage_name):
        def _cb(msg):
            self.on_tracked_stage(stage_name, msg)
        return _cb

    def _observe_stage(self, stage_name, header, objects, id_getter):
        self.global_stats['stage_messages'][stage_name] += 1
        stamp = _stamp_sec(header.stamp)
        mapped = []
        for index, obj in enumerate(objects):
            p = obj.kinematics.pose_with_covariance.pose.position
            xy = self._transform_to_map(float(p.x), float(p.y),
                                        header.frame_id)
            if xy is None:
                continue
            oid = id_getter(obj, index)
            mapped.append({
                'object_id': oid,
                'x': xy[0],
                'y': xy[1],
                'label': _label_from_object(obj),
                'shape': _shape_dict(obj),
            })

        for target in self.targets:
            stage_stats = self.target_stats[
                target['eid']]['stages'].setdefault(stage_name, {
                    'message_count': 0,
                    'nearest_sample_count': 0,
                    'within_match_count': 0,
                    'nearest_distance_m': None,
                    'nearest_object_id': '',
                    'nearest_label': -1,
                    'nearest_shape': None,
                    'distance_bins': Counter(),
                    'events': deque(maxlen=self.args.keep_events),
                })
            stage_stats['message_count'] += 1
            best = None
            for row in mapped:
                d = math.hypot(
                    row['x'] - target['map_x'],
                    row['y'] - target['map_y'])
                if best is None or d < best[0]:
                    best = (d, row)
            if best is None:
                continue
            d, row = best
            stage_stats['nearest_sample_count'] += 1
            stage_stats['distance_bins'][_distance_bin(d)] += 1
            prev = stage_stats['nearest_distance_m']
            if prev is None or d < prev:
                stage_stats['nearest_distance_m'] = float(d)
                stage_stats['nearest_object_id'] = row['object_id']
                stage_stats['nearest_label'] = int(row['label'])
                stage_stats['nearest_shape'] = row['shape']
            if d <= self.args.stage_match_distance:
                stage_stats['within_match_count'] += 1
                if ':' not in str(row['object_id']):
                    assoc = self._object_assoc(
                        target['eid'], row['object_id'])
                    self._update_object_assoc_pose(
                        assoc, d, row['x'], row['y'], row['shape'],
                        row['label'], stage_name)
                stage_stats['events'].append({
                    'stamp': stamp,
                    'stage': stage_name,
                    'object_id': row['object_id'],
                    'distance_m': float(d),
                    'x': float(row['x']),
                    'y': float(row['y']),
                    'label': int(row['label']),
                    'shape': row['shape'],
                })

    def on_detected_stage(self, stage_name, msg):
        self._observe_stage(
            stage_name, msg.header, msg.objects,
            lambda _obj, index: f'{stage_name}:{index}')

    def on_tracked_stage(self, stage_name, msg):
        self._observe_stage(
            stage_name, msg.header, msg.objects,
            lambda obj, _index: bytes(obj.object_id.uuid).hex())

    def on_tracked_objects(self, msg):
        self.global_stats['tracked_messages'] += 1
        stamp = _stamp_sec(msg.header.stamp)
        for obj in msg.objects:
            oid = bytes(obj.object_id.uuid).hex()
            p = obj.kinematics.pose_with_covariance.pose.position
            xy = self._transform_to_map(float(p.x), float(p.y),
                                        msg.header.frame_id)
            if xy is None:
                continue
            x, y = xy
            target, dist = self._nearest_target(x, y)
            target_eid = target['eid'] if target is not None else None
            self.object_positions[oid] = {
                'x': x,
                'y': y,
                'stamp': stamp,
                'label': _label_from_object(obj),
                'shape': _shape_dict(obj),
                'target_eid': target_eid,
                'target_distance_m': dist,
            }
            if target_eid is not None:
                stats = self.target_stats[target_eid]
                stats['tracked_observation_count'] += 1
                stats['object_ids'].add(oid)
                self._update_target_distance(target_eid, dist)
                assoc = self._object_assoc(target_eid, oid)
                assoc['tracked_count'] += 1
                self._update_object_assoc_pose(
                    assoc, dist, x, y, self.object_positions[oid]['shape'],
                    self.object_positions[oid]['label'], 'tracked')

    def on_debug(self, msg):
        self.global_stats['debug_messages'] += 1
        stamp = _stamp_sec(msg.header.stamp)
        for status in msg.status:
            object_id = status.name.split('/', 1)[0]
            values = _status_values(status)
            target_eid, dist, source = self._target_for_object(object_id)
            if target_eid is None:
                self.global_stats['unmatched_debug_statuses'] += 1
                continue
            stats = self.target_stats[target_eid]
            stats['object_ids'].add(object_id)
            stats['debug_status_count'] += 1
            assoc = self._object_assoc(target_eid, object_id)
            assoc['debug_count'] += 1
            if dist is not None:
                pos = self.object_positions.get(object_id, {})
                self._update_object_assoc_pose(
                    assoc, dist, pos.get('x'), pos.get('y'),
                    pos.get('shape'), pos.get('label'), 'debug')
            crop_key = _crop_key(values)
            stats['fovs'][crop_key] += 1
            assoc['fovs'][crop_key] += 1
            reason = str(status.message or '')
            klass = str(values.get('class', '') or '')
            selected = str(values.get('selected', '') or '')
            if reason == 'no_yolo_detection':
                stats['no_yolo_detection_count'] += 1
                assoc['no_yolo_count'] += 1
            else:
                stats['candidate_status_count'] += 1
                self.global_stats['candidate_statuses'] += 1
                if klass:
                    stats['candidate_classes'][klass] += 1
                    assoc['candidate_classes'][klass] += 1
                if reason == 'accepted':
                    stats['accepted_count'] += 1
                    assoc['accepted_count'] += 1
                    if klass:
                        stats['accepted_classes'][klass] += 1
                        assoc['accepted_classes'][klass] += 1
                else:
                    stats['rejected_count'] += 1
                    assoc['rejected_count'] += 1
                    stats['reject_reasons'][reason] += 1
                    assoc['reject_reasons'][reason] += 1
            if selected:
                stats['selected_count'] += 1
                assoc['selected_count'] += 1
                stats['selected_classes'][selected] += 1
                assoc['selected_classes'][selected] += 1
            self._update_target_distance(target_eid, dist)
            event = {
                'stamp': stamp,
                'source': source,
                'object_id': object_id,
                'status_name': status.name,
                'reason': reason,
                'class': klass,
                'conf': _as_float(values.get('conf'), None),
                'selected': selected,
                'selected_conf': _as_float(
                    values.get('selected_conf'), None),
                'target_distance_m': dist,
                'active_waypoint': self.active_waypoint,
            }
            stats['events'].append(event)
            assoc['events'].append(event)

    def on_fine_classes(self, msg):
        self.global_stats['fine_class_messages'] += 1
        stamp = _stamp_sec(msg.header.stamp)
        for status in msg.status:
            object_id = status.name
            if not object_id:
                continue
            klass = str(status.message or '')
            values = _status_values(status)
            conf = _as_float(values.get('conf'), 0.0)
            if not klass:
                self.fine_classes.pop(object_id, None)
                continue
            self.fine_classes[object_id] = {
                'class': klass,
                'conf': conf,
                'stamp': stamp,
            }
            target_eid, dist, source = self._target_for_object(object_id)
            if target_eid is None:
                self.global_stats['unmatched_fine_class_statuses'] += 1
                continue
            stats = self.target_stats[target_eid]
            stats['object_ids'].add(object_id)
            stats['fine_class_count'] += 1
            stats['fine_classes'][klass] += 1
            assoc = self._object_assoc(target_eid, object_id)
            assoc['fine_count'] += 1
            assoc['fine_classes'][klass] += 1
            if dist is not None:
                pos = self.object_positions.get(object_id, {})
                self._update_object_assoc_pose(
                    assoc, dist, pos.get('x'), pos.get('y'),
                    pos.get('shape'), pos.get('label'), 'fine_class')
            self._update_target_distance(target_eid, dist)
            event = {
                'stamp': stamp,
                'source': source,
                'object_id': object_id,
                'reason': 'fine_class',
                'class': klass,
                'conf': conf,
                'target_distance_m': dist,
                'active_waypoint': self.active_waypoint,
            }
            stats['events'].append(event)
            assoc['events'].append(event)

    def on_tracker_debug(self, msg):
        stamp = _stamp_sec(msg.header.stamp)
        for status in msg.status:
            values = _status_values(status)
            frame_id = values.get('frame_id') or msg.header.frame_id
            xy = self._transform_to_map(
                _as_float(values.get('x'), 0.0),
                _as_float(values.get('y'), 0.0),
                frame_id)
            if xy is None:
                continue
            target, dist = self._nearest_target(xy[0], xy[1])
            if target is None:
                continue
            stats = self.target_stats[target['eid']]
            stats['tracker_debug_status_count'] += 1
            assoc = self._object_assoc(target['eid'], status.name)
            assoc['tracker_debug_count'] += 1
            self._update_object_assoc_pose(
                assoc, dist, xy[0], xy[1], None,
                int(_as_float(values.get('label'), -1)), 'tracker_debug')
            self._update_target_distance(target['eid'], dist)
            prev = stats['tracker_debug_nearest_distance_m']
            if prev is None or dist < prev:
                stats['tracker_debug_nearest_distance_m'] = float(dist)
            if dist <= self.args.stage_match_distance:
                stats['tracker_debug_near_count'] += 1
                reason = str(status.message or '')
                stats['tracker_debug_reasons'][reason] += 1
                assoc['tracker_reasons'][reason] += 1
                event = {
                    'stamp': stamp,
                    'object_id': status.name,
                    'reason': reason,
                    'distance_m': float(dist),
                    'x': float(xy[0]),
                    'y': float(xy[1]),
                    'hits': int(_as_float(values.get('hits'), 0)),
                    'existence': _as_float(values.get('existence'), 0.0),
                    'is_stationary':
                        str(values.get('is_stationary', '')).lower() == 'true',
                    'map_blocked':
                        str(values.get('map_blocked', '')).lower() == 'true',
                    'wall_margin_cells':
                        int(_as_float(values.get('wall_margin_cells'), 0)),
                    'speed': _as_float(values.get('speed'), 0.0),
                    'label': int(_as_float(values.get('label'), -1)),
                }
                stats['tracker_debug_events'].append(event)
                assoc['events'].append(event)

    def on_waypoint_status(self, msg):
        self.global_stats['waypoint_status_messages'] += 1
        text = str(msg.data or '')
        self.waypoint_statuses.append({
            'wall_elapsed_sec': time.monotonic() - self.started_wall,
            'text': text,
        })
        m = re.search(r'heading to waypoint #(\d+)', text)
        if m:
            self.active_waypoint = int(m.group(1))
            self.active_target_eid = self.target_by_waypoint.get(
                self.active_waypoint)
            return
        if 'lap finished' in text or 'mission ' in text:
            self.active_waypoint = None
            self.active_target_eid = (
                self.targets[0]['eid']
                if self.args.active_waypoint_fallback and len(self.targets) == 1
                else None)

    def _serializable_report(self):
        targets = []
        for eid, stats in self.target_stats.items():
            row = {
                'target': stats['target'],
                'debug_status_count': stats['debug_status_count'],
                'candidate_status_count': stats['candidate_status_count'],
                'no_yolo_detection_count': stats['no_yolo_detection_count'],
                'accepted_count': stats['accepted_count'],
                'rejected_count': stats['rejected_count'],
                'selected_count': stats['selected_count'],
                'fine_class_count': stats['fine_class_count'],
                'tracked_observation_count':
                    stats['tracked_observation_count'],
                'nearest_tracked_distance_m':
                    stats['nearest_tracked_distance_m'],
                'object_ids': sorted(stats['object_ids']),
                'candidate_classes': _counter_to_dict(
                    stats['candidate_classes']),
                'accepted_classes': _counter_to_dict(
                    stats['accepted_classes']),
                'selected_classes': _counter_to_dict(
                    stats['selected_classes']),
                'fine_classes': _counter_to_dict(stats['fine_classes']),
                'reject_reasons': _counter_to_dict(stats['reject_reasons']),
                'fovs': _counter_to_dict(stats['fovs']),
                'tracker_debug_status_count':
                    stats['tracker_debug_status_count'],
                'tracker_debug_near_count':
                    stats['tracker_debug_near_count'],
                'tracker_debug_nearest_distance_m':
                    stats['tracker_debug_nearest_distance_m'],
                'tracker_debug_reasons': _counter_to_dict(
                    stats['tracker_debug_reasons']),
                'tracker_debug_events':
                    list(stats['tracker_debug_events']),
                'object_associations':
                    self._serializable_object_associations(stats),
                'stages': {
                    name: {
                        'message_count':
                            int(stage_stats['message_count']),
                        'nearest_sample_count':
                            int(stage_stats['nearest_sample_count']),
                        'within_match_count':
                            int(stage_stats['within_match_count']),
                        'nearest_distance_m':
                            stage_stats['nearest_distance_m'],
                        'nearest_object_id':
                            stage_stats['nearest_object_id'],
                        'nearest_label':
                            int(stage_stats['nearest_label']),
                        'nearest_shape': stage_stats['nearest_shape'],
                        'distance_bins': _counter_to_dict(
                            stage_stats['distance_bins']),
                        'events': list(stage_stats['events']),
                    }
                    for name, stage_stats in stats['stages'].items()
                },
                'events': list(stats['events']),
            }
            targets.append(row)
        return {
            'inputs': {
                'target_waypoints': self.args.target_waypoints,
                'debug_topic': self.args.debug_topic,
                'fine_class_topic': self.args.fine_class_topic,
                'tracked_topic': self.args.tracked_topic,
                'waypoint_status_topic': self.args.waypoint_status_topic,
                'tracker_debug_topic': self.args.tracker_debug_topic,
                'detected_stage_topics': dict(self.detected_stage_topics),
                'tracked_stage_topics': dict(self.tracked_stage_topics),
                'target_match_distance': self.args.target_match_distance,
                'stage_match_distance': self.args.stage_match_distance,
                'active_waypoint_fallback':
                    self.args.active_waypoint_fallback,
            },
            'summary': {
                **self.global_stats,
                'stage_messages': _counter_to_dict(
                    self.global_stats['stage_messages']),
                'target_count': len(self.targets),
                'targets_with_debug': sum(
                    1 for row in targets
                    if row['debug_status_count'] > 0),
                'targets_with_accepted': sum(
                    1 for row in targets if row['accepted_count'] > 0),
                'targets_with_fine_class': sum(
                    1 for row in targets if row['fine_class_count'] > 0),
                'wall_elapsed_sec': round(
                    time.monotonic() - self.started_wall, 3),
            },
            'targets': targets,
            'waypoint_statuses': list(self.waypoint_statuses),
        }

    def write_reports(self):
        report = self._serializable_report()
        prefix = self.args.out_prefix
        out_dir = os.path.dirname(prefix)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(prefix + '.json', 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
            f.write('\n')
        with open(prefix + '.csv', 'w', newline='', encoding='utf-8') as f:
            fields = [
                'target', 'waypoint_index', 'tracked_observations',
                'debug_statuses', 'no_yolo', 'accepted', 'rejected',
                'selected', 'fine_class', 'nearest_tracked_m',
                'accepted_classes', 'selected_classes', 'fine_classes',
                'reject_reasons', 'tracker_debug_reasons',
                'stage_nearest_m', 'object_ids', 'object_associations',
            ]
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for row in report['targets']:
                target = row['target']
                writer.writerow({
                    'target': target['eid'],
                    'waypoint_index': target.get('waypoint_index'),
                    'tracked_observations':
                        row['tracked_observation_count'],
                    'debug_statuses': row['debug_status_count'],
                    'no_yolo': row['no_yolo_detection_count'],
                    'accepted': row['accepted_count'],
                    'rejected': row['rejected_count'],
                    'selected': row['selected_count'],
                    'fine_class': row['fine_class_count'],
                    'nearest_tracked_m': (
                        '' if row['nearest_tracked_distance_m'] is None
                        else f"{row['nearest_tracked_distance_m']:.3f}"),
                    'accepted_classes': ','.join(
                        f'{k}:{v}' for k, v in
                        row['accepted_classes'].items()),
                    'selected_classes': ','.join(
                        f'{k}:{v}' for k, v in
                        row['selected_classes'].items()),
                    'fine_classes': ','.join(
                        f'{k}:{v}' for k, v in row['fine_classes'].items()),
                    'reject_reasons': ','.join(
                        f'{k}:{v}' for k, v in
                        row['reject_reasons'].items()),
                    'tracker_debug_reasons': ','.join(
                        f'{k}:{v}' for k, v in
                        row['tracker_debug_reasons'].items()),
                    'stage_nearest_m': ','.join(
                        f"{name}:{stage.get('nearest_distance_m'):.3f}"
                        for name, stage in row['stages'].items()
                        if stage.get('nearest_distance_m') is not None),
                    'object_ids': ','.join(row['object_ids']),
                    'object_associations': ';'.join(
                        self._format_object_assoc_csv(a)
                        for a in row.get('object_associations', [])[:8]),
                })
        self._write_markdown(prefix + '.md', report)
        self.last_write_wall = time.monotonic()

    @staticmethod
    def _format_object_assoc_csv(assoc):
        nearest = assoc.get('nearest_distance_m')
        nearest_txt = '' if nearest is None else f'{nearest:.3f}'
        classes = _top(Counter(assoc.get('selected_classes', {})), 2)
        if not classes:
            classes = _top(Counter(assoc.get('accepted_classes', {})), 2)
        return (
            f"{assoc.get('object_id', '')[:8]}@{nearest_txt}"
            f" t{assoc.get('tracked_count', 0)}"
            f" d{assoc.get('debug_count', 0)}"
            f" f{assoc.get('fine_count', 0)}"
            f" {classes}")

    @staticmethod
    def _write_markdown(path, report):
        s = report['summary']
        lines = [
            '# Recognition Debug Recorder',
            '',
            '## Summary',
            '',
            f"- target_count: `{s['target_count']}`",
            f"- targets_with_debug: `{s['targets_with_debug']}`",
            f"- targets_with_accepted: `{s['targets_with_accepted']}`",
            f"- targets_with_fine_class: `{s['targets_with_fine_class']}`",
            f"- unmatched_debug_statuses: `{s['unmatched_debug_statuses']}`",
            f"- unmatched_fine_class_statuses: `{s['unmatched_fine_class_statuses']}`",
            f"- tf_failures: `{s['tf_failures']}`",
            f"- elapsed: `{s['wall_elapsed_sec']}s`",
            '',
            '## Targets',
            '',
            ('| target | wp | tracked | debug | no_yolo | accepted | '
             'selected | fine | nearest | accepted classes | selected classes | '
             'fine classes | reject reasons | tracker reasons |'),
            '|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---|---|---|---|',
        ]
        for row in report['targets']:
            target = row['target']
            nearest = row['nearest_tracked_distance_m']
            nearest_txt = '' if nearest is None else f'{nearest:.2f}'
            lines.append(
                f"| `{target['eid']}` | {target.get('waypoint_index')} "
                f"| {row['tracked_observation_count']} "
                f"| {row['debug_status_count']} "
                f"| {row['no_yolo_detection_count']} "
                f"| {row['accepted_count']} "
                f"| {row['selected_count']} "
                f"| {row['fine_class_count']} "
                f"| {nearest_txt} "
                f"| {_top(Counter(row['accepted_classes']))} "
                f"| {_top(Counter(row['selected_classes']))} "
                f"| {_top(Counter(row['fine_classes']))} "
                f"| {_top(Counter(row['reject_reasons']))} "
                f"| {_top(Counter(row['tracker_debug_reasons']))} |")
        lines += [
            '',
            '## Object Associations',
            '',
            ('| target | object | nearest | tracked | stages | debug | '
             'accepted | selected | fine | tracker | shape | selected classes | '
             'accepted classes | reject reasons | tracker reasons |'),
            '|---|---|---:|---:|---|---:|---:|---:|---:|---:|---|---|---|---|---|',
        ]
        for row in report['targets']:
            target = row['target']
            associations = row.get('object_associations', [])
            if not associations:
                lines.append(
                    f"| `{target['eid']}` |  |  | 0 |  | 0 | 0 | 0 | 0 | 0 |  |  |  |  |  |")
                continue
            for assoc in associations[:8]:
                nearest = assoc.get('nearest_distance_m')
                nearest_txt = '' if nearest is None else f'{nearest:.2f}'
                stages = ', '.join(
                    f'{k}:{v}' for k, v in
                    assoc.get('stage_counts', {}).items())
                shape = (
                    assoc.get('nearest_shape') or assoc.get('last_shape') or {})
                lines.append(
                    f"| `{target['eid']}` "
                    f"| `{assoc.get('object_id', '')[:8]}` "
                    f"| {nearest_txt} "
                    f"| {assoc.get('tracked_count', 0)} "
                    f"| {stages} "
                    f"| {assoc.get('debug_count', 0)} "
                    f"| {assoc.get('accepted_count', 0)} "
                    f"| {assoc.get('selected_count', 0)} "
                    f"| {assoc.get('fine_count', 0)} "
                    f"| {assoc.get('tracker_debug_count', 0)} "
                    f"| {_shape_txt(shape)} "
                    f"| {_top(Counter(assoc.get('selected_classes', {})))} "
                    f"| {_top(Counter(assoc.get('accepted_classes', {})))} "
                    f"| {_top(Counter(assoc.get('reject_reasons', {})))} "
                    f"| {_top(Counter(assoc.get('tracker_reasons', {})))} |")
        if report['waypoint_statuses']:
            lines += ['', '## Waypoint Statuses', '']
            for row in report['waypoint_statuses'][-20:]:
                lines.append(
                    f"- `{row['wall_elapsed_sec']:.1f}s` {row['text']}")
        lines += [
            '',
            '## Pipeline Stages',
            '',
            ('| target | stage | messages | samples | within match | nearest '
             '| nearest shape | nearest label | distance bins |'),
            '|---|---|---:|---:|---:|---:|---|---:|---|',
        ]
        for row in report['targets']:
            target = row['target']
            for stage_name, stage in row['stages'].items():
                nearest = stage.get('nearest_distance_m')
                nearest_txt = '' if nearest is None else f'{nearest:.2f}'
                shape = stage.get('nearest_shape') or {}
                shape_txt = ''
                if shape:
                    shape_txt = (
                        f"{shape.get('x', 0.0):.2f}x"
                        f"{shape.get('y', 0.0):.2f}x"
                        f"{shape.get('z', 0.0):.2f}")
                bins = ', '.join(
                    f'{k}:{v}' for k, v in
                    stage.get('distance_bins', {}).items())
                lines.append(
                    f"| `{target['eid']}` | `{stage_name}` "
                    f"| {stage.get('message_count', 0)} "
                    f"| {stage.get('nearest_sample_count', 0)} "
                    f"| {stage.get('within_match_count', 0)} "
                    f"| {nearest_txt} "
                    f"| {shape_txt} "
                    f"| {stage.get('nearest_label', -1)} "
                    f"| {bins} |")
        lines += [
            '',
            '## Tracker Debug',
            '',
            ('| target | statuses | near | nearest | reasons | recent near '
             'events |'),
            '|---|---:|---:|---:|---|---|',
        ]
        for row in report['targets']:
            target = row['target']
            nearest = row.get('tracker_debug_nearest_distance_m')
            nearest_txt = '' if nearest is None else f'{nearest:.2f}'
            events = []
            for ev in row.get('tracker_debug_events', [])[-5:]:
                events.append(
                    f"{ev.get('reason')}#{ev.get('object_id', '')[:8]}"
                    f" d={ev.get('distance_m', 0.0):.2f}"
                    f" h={ev.get('hits', 0)}"
                    f" m={ev.get('wall_margin_cells', 0)}")
            lines.append(
                f"| `{target['eid']}` "
                f"| {row.get('tracker_debug_status_count', 0)} "
                f"| {row.get('tracker_debug_near_count', 0)} "
                f"| {nearest_txt} "
                f"| {_top(Counter(row.get('tracker_debug_reasons', {})))} "
                f"| {'; '.join(events)} |")
        with open(path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines) + '\n')

    def on_timer(self):
        self.write_reports()
        if self.args.duration_sec > 0.0 and \
                (time.monotonic() - self.started_wall) >= self.args.duration_sec:
            self.get_logger().info('duration reached; stopping recorder')
            self.stop_requested = True


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--target-waypoints', required=True)
    parser.add_argument('--out-prefix', required=True)
    parser.add_argument('--duration-sec', type=float, default=0.0)
    parser.add_argument('--write-period-sec', type=float, default=2.0)
    parser.add_argument('--target-match-distance', type=float, default=1.2)
    parser.add_argument('--stage-match-distance', type=float, default=2.0)
    parser.add_argument('--active-waypoint-fallback', action='store_true',
                        help='attribute unmatched debug entries to the active debug waypoint target')
    parser.add_argument('--keep-events', type=int, default=40)
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--debug-topic',
                        default='/perception/object_classifier/debug')
    parser.add_argument('--fine-class-topic',
                        default='/perception/object_fine_classes')
    parser.add_argument('--tracked-topic',
                        default='/perception/tracked_objects_classified')
    parser.add_argument('--tracker-debug-topic',
                        default='/perception/object_tracker/debug')
    parser.add_argument(
        '--detected-stage-topic', action='append',
        default=[
            'detected:/perception/detected_objects',
            'shaped:/perception/detected_objects_shaped',
            'merged:/perception/detected_objects_merged',
            'in_map:/perception/detected_objects_in_map',
        ],
        help='NAME:/topic for DetectedObjects stage diagnostics')
    parser.add_argument(
        '--tracked-stage-topic', action='append',
        default=['tracked_raw:/perception/tracked_objects'],
        help='NAME:/topic for TrackedObjects stage diagnostics')
    parser.add_argument('--waypoint-status-topic',
                        default='/waypoint_nav/status')
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = RecognitionDebugRecorder(args)

    def _stop(_signum, _frame):
        node.stop_requested = True

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.2)
            if args.duration_sec > 0.0 and \
                    (time.monotonic() - node.started_wall) >= args.duration_sec:
                node.stop_requested = True
    finally:
        node.write_reports()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
