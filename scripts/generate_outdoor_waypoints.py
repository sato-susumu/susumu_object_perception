#!/usr/bin/env python3
"""Outdoor-specific wrapper around generate_waypoints.py.

The core waypoint generator is shared, but outdoor maps should not inherit the
indoor defaults by accident. This wrapper keeps the outdoor patrol defaults in
one place while still writing the same waypoint YAML/PNG format consumed by
waypoint_nav_node.py.

Outdoor patrol maps are produced by slam_toolbox from sensor data. Do not feed
world-derived ground-truth maps into this wrapper except for a deliberately
separate diagnostic experiment. The default radius limit avoids selecting free
cells that leaked past the intended patrol region in an imperfect SLAM map.

Unlike indoor patrol, outdoor patrol covers larger open roads and plazas. The
defaults therefore use coarser route goals: Nav2 should follow road-scale
segments instead of stopping every few meters at dense coverage samples.
"""

import argparse
import runpy
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--spacing', type=float, default=4.0)
    parser.add_argument('--clearance', type=float, default=0.75)
    parser.add_argument('--connect-clearance', type=float, default=0.35)
    parser.add_argument(
        '--route-clearance', type=float, default=None,
        help='experimental edge clearance [m]; default uses --connect-clearance')
    parser.add_argument(
        '--edge-clearance', type=float, default=None,
        help='desired soft clearance for route edge ordering [m]')
    parser.add_argument(
        '--edge-clearance-weight', type=float, default=0.0,
        help='soft penalty weight for low-clearance route edges')
    parser.add_argument(
        '--edge-risk-report', default='',
        help='optional prefix for route edge risk CSV/MD reports')
    parser.add_argument(
        '--hazard-file', action='append', default=[],
        help='JSON/YAML/CSV hazard points from previous outdoor runs')
    parser.add_argument(
        '--hazard-radius', type=float, default=1.5,
        help='default hazard radius [m] when the file has no radius')
    parser.add_argument(
        '--strict-hazard-file', action='store_true',
        help='fail instead of warning when a hazard file cannot be read')
    parser.add_argument(
        '--require-hazards', action='store_true',
        help='fail when hazard files are provided but no usable hazards are on the map')
    parser.add_argument('--max-waypoints', type=int, default=40)
    parser.add_argument('--max-segment-length', type=float, default=4.0)
    parser.add_argument('--limit-radius', type=float, default=14.0)
    parser.add_argument('--limit-center-x', type=float, default=0.0)
    parser.add_argument('--limit-center-y', type=float, default=0.0)
    parser.add_argument('--object-viewpoints', type=int, default=0)
    parser.add_argument('--no-png', action='store_true')
    args, extra = parser.parse_known_args()

    target = Path(__file__).with_name('generate_waypoints.py')
    argv = [
        str(target),
        '--map', args.map,
        '--out', args.out,
        '--spacing', str(args.spacing),
        '--clearance', str(args.clearance),
        '--connect-clearance', str(args.connect_clearance),
        '--max-waypoints', str(args.max_waypoints),
        '--max-segment-length', str(args.max_segment_length),
        '--limit-radius', str(args.limit_radius),
        '--limit-center-x', str(args.limit_center_x),
        '--limit-center-y', str(args.limit_center_y),
        '--object-viewpoints', str(args.object_viewpoints),
    ]
    if args.route_clearance is not None:
        argv += ['--route-clearance', str(args.route_clearance)]
    if args.edge_clearance is not None:
        argv += ['--edge-clearance', str(args.edge_clearance)]
    if args.edge_clearance_weight > 0.0:
        argv += ['--edge-clearance-weight', str(args.edge_clearance_weight)]
    if args.edge_risk_report:
        argv += ['--edge-risk-report', args.edge_risk_report]
    for path in args.hazard_file:
        argv += ['--hazard-file', path]
    if args.hazard_file:
        argv += ['--hazard-radius', str(args.hazard_radius)]
    if args.strict_hazard_file:
        argv.append('--strict-hazard-file')
    if args.require_hazards:
        argv.append('--require-hazards')
    if args.no_png:
        argv.append('--no-png')
    argv.extend(extra)
    sys.argv = argv
    runpy.run_path(str(target), run_name='__main__')


if __name__ == '__main__':
    main()
