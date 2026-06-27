# Waypoint Check

- schema_version: `3`
- validation_passed: `true`
- map: `outputs/mapping_indoor/indoor.yaml`
- map_image: `outputs/mapping_indoor/indoor.pgm`
- waypoints: `outputs/waypoint_generation/indoor_waypoints.yaml`
- map_sha256: `e908dc8c8796c0c5fbe5eb8bddc7c4dfc3587ae84e33d18c8ed1b450179781a9`
- map_image_sha256: `a894e9e33265b2d81500fbf71e3ff81d683f68b4fd7c1032a3e98c34a459513b`
- waypoints_sha256: `0fe8e5e328f022dc49dd7e3e2c1085f786d160afb8c3802fc98eab1bf7f2fe93`
- waypoint_count: `9`

| metric | value |
|---|---:|
| min_clearance_m | 0.600 |
| mean_clearance_m | 1.020 |
| near_clearance_count | 0 |
| coverage_mean_m | 0.611 |
| coverage_max_m | 1.601 |
| thin_coverage_cells | 0 |
| route_unreachable_edges | 0 |
| route_over_jump_edges | 0 |
| route_max_geodesic_m | 6.507 |

## Worst Clearances

| index | clearance[m] | x | y |
|---:|---:|---:|---:|
| 3 | 0.600 | 3.075 | 3.115 |
| 8 | 0.600 | 3.075 | -3.485 |
| 4 | 0.950 | 2.725 | 2.015 |
| 7 | 0.950 | 1.925 | -3.985 |
| 1 | 1.006 | -0.125 | 2.015 |
| 0 | 1.050 | -0.125 | 0.015 |
| 6 | 1.100 | 1.575 | -2.335 |
| 5 | 1.373 | 0.275 | -2.335 |

## Longest Route Edges

| edge | geodesic[m] | straight[m] | from | to |
|---|---:|---:|---|---|
| 4->5 | 6.507 | 4.992 | (2.725,2.015) | (0.275,-2.335) |
| 0->1 | 2.000 | 2.000 | (-0.125,0.015) | (-0.125,2.015) |
| 1->2 | 1.926 | 1.851 | (-0.125,2.015) | (1.325,3.165) |
| 6->7 | 1.795 | 1.687 | (1.575,-2.335) | (1.925,-3.985) |
| 2->3 | 1.771 | 1.751 | (1.325,3.165) | (3.075,3.115) |
| 7->8 | 1.357 | 1.254 | (1.925,-3.985) | (3.075,-3.485) |
| 5->6 | 1.300 | 1.300 | (0.275,-2.335) | (1.575,-2.335) |
| 3->4 | 1.245 | 1.154 | (3.075,3.115) | (2.725,2.015) |
