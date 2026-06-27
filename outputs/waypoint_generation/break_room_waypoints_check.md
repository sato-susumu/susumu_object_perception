# Waypoint Check

- schema_version: `3`
- validation_passed: `true`
- map: `outputs/mapping_indoor/break_room.yaml`
- map_image: `outputs/mapping_indoor/break_room.pgm`
- waypoints: `outputs/waypoint_generation/break_room_waypoints.yaml`
- map_sha256: `f3e1776103ad0a564a660b445eb6b56af93d44a9e443d1a16f7e8c095fc6a95a`
- map_image_sha256: `e122f39e97f8187e5a4ea45f09b3538085370a23e7cda5834f0c38b187b78129`
- waypoints_sha256: `ffaa7c5b860803e6fa2487d0b052541f65020bbd9cb07bf717c28c5d0134672f`
- waypoint_count: `19`

| metric | value |
|---|---:|
| min_clearance_m | 0.610 |
| mean_clearance_m | 1.009 |
| near_clearance_count | 0 |
| coverage_mean_m | 0.572 |
| coverage_max_m | 1.736 |
| thin_coverage_cells | 0 |
| route_unreachable_edges | 0 |
| route_over_jump_edges | 0 |
| route_max_geodesic_m | 4.849 |

## Worst Clearances

| index | clearance[m] | x | y |
|---:|---:|---:|---:|
| 14 | 0.610 | -0.725 | 3.595 |
| 8 | 0.650 | 4.475 | -0.155 |
| 6 | 0.762 | 0.775 | -1.655 |
| 17 | 0.778 | -5.175 | 0.545 |
| 0 | 0.781 | -0.575 | 1.395 |
| 5 | 0.800 | -2.275 | -1.655 |
| 3 | 0.850 | -2.225 | 1.145 |
| 4 | 0.850 | -2.225 | -0.155 |

## Longest Route Edges

| edge | geodesic[m] | straight[m] | from | to |
|---|---:|---:|---|---|
| 14->15 | 4.849 | 4.627 | (-0.725,3.595) | (-5.325,3.095) |
| 7->8 | 4.528 | 3.700 | (0.775,-0.155) | (4.475,-0.155) |
| 5->6 | 3.050 | 3.050 | (-2.275,-1.655) | (0.775,-1.655) |
| 10->11 | 2.514 | 2.220 | (3.675,1.395) | (4.975,3.195) |
| 12->13 | 2.321 | 2.301 | (3.375,3.145) | (1.075,3.195) |
| 15->16 | 1.995 | 1.883 | (-5.325,3.095) | (-5.675,1.245) |
| 13->14 | 1.966 | 1.844 | (1.075,3.195) | (-0.725,3.595) |
| 17->18 | 1.921 | 1.901 | (-5.175,0.545) | (-5.225,-1.355) |
