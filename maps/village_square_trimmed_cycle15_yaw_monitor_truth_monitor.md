# Live SLAM Truth Monitor

- gps_topic: `/TurtleBot3Burger/gps`
- gps_type: `geometry_msgs/msg/PointStamped`
- imu_topic: `/imu`
- estimate_frame: `map`
- robot_frame: `base_footprint`
- samples: `230`
- events: `15`
- gps_path_length_m: `17.126`
- estimate_path_length_m: `25.373`
- max_aligned_error_m: `2.578`
- max_heading_error_deg: `60.27`
- max_yaw_error_deg: `108.67`

## Events

| idx | gps rel xy | estimate xy | aligned err | heading err | yaw err | reason |
|---:|---|---|---:|---:|---:|---|
| 101 | [6.062,-2.279] | [6.055,-2.326] | 0.035 | 0.95 | 11.16 | yaw_error 11.2deg > 8.0deg |
| 111 | [6.461,-1.327] | [7.114,-2.154] | 0.921 | 60.27 | 54.94 | aligned_error 0.92m > 0.60m; heading_error 60.3deg > 30.0deg; yaw_error 54.9deg > 8.0deg |
| 118 | [7.149,-0.587] | [8.122,-2.258] | 1.457 | 48.39 | 54.23 | aligned_error 1.46m > 0.60m; heading_error 48.4deg > 30.0deg; yaw_error 54.2deg > 8.0deg |
| 127 | [8.155,-0.367] | [8.880,-2.939] | 1.549 | 44.18 | 54.41 | aligned_error 1.55m > 0.60m; heading_error 44.2deg > 30.0deg; yaw_error 54.4deg > 8.0deg |
| 134 | [8.588,-0.434] | [9.070,-3.323] | 1.367 |  | 54.88 | aligned_error 1.37m > 0.60m; yaw_error 54.9deg > 8.0deg |
| 143 | [8.017,-1.181] | [8.158,-3.307] | 0.540 | 38.23 | 53.59 | heading_error 38.2deg > 30.0deg; yaw_error 53.6deg > 8.0deg |
| 153 | [7.043,-1.079] | [7.689,-2.490] | 0.494 |  | 54.83 | yaw_error 54.8deg > 8.0deg |
| 162 | [6.115,-0.527] | [7.591,-1.398] | 1.130 | 38.93 | 63.54 | aligned_error 1.13m > 0.60m; heading_error 38.9deg > 30.0deg; yaw_error 63.5deg > 8.0deg |
| 171 | [6.096,-0.507] | [7.440,-0.592] | 1.351 |  | 76.85 | aligned_error 1.35m > 0.60m; yaw_error 76.9deg > 8.0deg |
| 181 | [6.096,-0.507] | [7.362,-0.028] | 1.664 |  | 96.05 | aligned_error 1.66m > 0.60m; yaw_error 96.1deg > 8.0deg |
| 191 | [6.096,-0.507] | [7.263,0.558] | 2.044 |  | 91.17 | aligned_error 2.04m > 0.60m; yaw_error 91.2deg > 8.0deg |
| 201 | [6.096,-0.507] | [7.288,1.246] | 2.564 |  | 70.96 | aligned_error 2.56m > 0.60m; yaw_error 71.0deg > 8.0deg |
| 211 | [6.096,-0.508] | [7.359,-0.999] | 0.714 |  | 60.32 | aligned_error 0.71m > 0.60m; yaw_error 60.3deg > 8.0deg |
| 221 | [6.096,-0.508] | [7.049,-0.363] | 0.844 |  | 66.04 | aligned_error 0.84m > 0.60m; yaw_error 66.0deg > 8.0deg |
| 228 | [6.096,-0.508] | [6.764,0.422] | 1.501 |  | 67.84 | aligned_error 1.50m > 0.60m; yaw_error 67.8deg > 8.0deg |
