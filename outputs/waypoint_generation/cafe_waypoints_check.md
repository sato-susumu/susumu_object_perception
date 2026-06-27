# Waypoint Check

- schema_version: `3`
- validation_passed: `true`
- map: `outputs/mapping_indoor/cafe.yaml`
- map_image: `outputs/mapping_indoor/cafe.pgm`
- waypoints: `outputs/waypoint_generation/cafe_waypoints.yaml`
- map_sha256: `3b8793fea104f196dc52189f518a4751705d568cf3cc34b188bbf67007253bf3`
- map_image_sha256: `c487dc620954af767c654243319daea8f9071fd4460f9dc931615a228e2e241f`
- waypoints_sha256: `9d29eae2c0254cad439863ee9e9c2558c8ba4517813fca370a19d2d5e1fe4e1b`
- waypoint_count: `40`

| metric | value |
|---|---:|
| min_clearance_m | 0.600 |
| mean_clearance_m | 1.554 |
| near_clearance_count | 0 |
| coverage_mean_m | 0.801 |
| coverage_max_m | 2.022 |
| thin_coverage_cells | 0 |
| route_unreachable_edges | 0 |
| route_over_jump_edges | 0 |
| route_max_geodesic_m | 5.630 |

## Worst Clearances

| index | clearance[m] | x | y |
|---:|---:|---:|---:|
| 28 | 0.600 | 3.615 | -6.475 |
| 17 | 0.618 | 1.265 | 10.525 |
| 16 | 0.778 | 1.315 | 6.925 |
| 10 | 0.922 | -2.385 | 5.425 |
| 29 | 0.955 | 2.965 | -8.125 |
| 18 | 1.001 | 2.965 | 8.475 |
| 36 | 1.026 | -2.385 | -8.125 |
| 35 | 1.050 | -0.935 | -9.825 |

## Longest Route Edges

| edge | geodesic[m] | straight[m] | from | to |
|---|---:|---:|---|---|
| 17->18 | 5.630 | 2.663 | (1.265,10.525) | (2.965,8.475) |
| 26->27 | 4.922 | 4.732 | (-0.835,-3.475) | (2.115,-7.175) |
| 6->7 | 3.824 | 3.700 | (-3.935,-1.325) | (-3.935,2.375) |
| 16->17 | 3.621 | 3.600 | (1.315,6.925) | (1.265,10.525) |
| 14->15 | 3.150 | 3.150 | (0.565,2.425) | (0.565,5.575) |
| 21->22 | 3.000 | 3.000 | (2.115,3.875) | (2.115,0.875) |
| 38->39 | 2.991 | 2.952 | (-3.935,-6.675) | (-3.835,-9.625) |
| 25->26 | 2.724 | 2.617 | (1.765,-3.175) | (-0.835,-3.475) |
