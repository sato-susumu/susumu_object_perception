# Live SLAM Truth Monitor

- gps_topic: `/TurtleBot3Burger/gps`
- gps_type: `geometry_msgs/msg/PointStamped`
- imu_topic: `/imu`
- estimate_frame: `map`
- robot_frame: `base_footprint`
- samples: `186`
- events: `12`
- gps_path_length_m: `12.371`
- estimate_path_length_m: `14.128`
- max_aligned_error_m: `1.192`
- max_heading_error_deg: `1.68`
- max_yaw_error_deg: `111.92`

## Events

| idx | gps rel xy | estimate xy | aligned err | heading err | yaw err | reason |
|---:|---|---|---:|---:|---:|---|
| 93 | [6.039,-2.214] | [6.041,-2.228] | 0.012 |  | 9.25 | yaw_error 9.3deg > 8.0deg |
| 102 | [6.290,-2.769] | [6.808,-1.972] | 0.872 |  | 83.53 | aligned_error 0.87m > 0.60m; yaw_error 83.5deg > 8.0deg |
| 111 | [6.359,-3.569] | [7.609,-1.984] | 1.615 |  | 83.67 | aligned_error 1.61m > 0.60m; yaw_error 83.7deg > 8.0deg |
| 120 | [6.096,-3.883] | [8.000,-2.423] | 1.703 |  | 97.29 | aligned_error 1.70m > 0.60m; yaw_error 97.3deg > 8.0deg |
| 129 | [6.106,-3.888] | [8.004,-2.453] | 1.469 |  | 92.18 | aligned_error 1.47m > 0.60m; yaw_error 92.2deg > 8.0deg |
| 137 | [6.105,-3.878] | [8.004,-2.459] | 1.329 |  | 93.89 | aligned_error 1.33m > 0.60m; yaw_error 93.9deg > 8.0deg |
| 143 | [6.106,-3.877] | [8.004,-2.459] | 1.245 |  | 95.06 | aligned_error 1.25m > 0.60m; yaw_error 95.1deg > 8.0deg |
| 150 | [6.106,-3.878] | [8.004,-2.459] | 1.164 |  | 95.06 | aligned_error 1.16m > 0.60m; yaw_error 95.1deg > 8.0deg |
| 160 | [6.066,-3.826] | [7.801,-2.431] | 0.926 |  | 52.63 | aligned_error 0.93m > 0.60m; yaw_error 52.6deg > 8.0deg |
| 169 | [6.032,-3.671] | [7.264,-2.237] | 0.506 |  | 59.16 | yaw_error 59.2deg > 8.0deg |
| 177 | [6.030,-3.515] | [6.737,-1.984] | 0.453 |  | 36.42 | yaw_error 36.4deg > 8.0deg |
| 185 | [6.031,-3.377] | [6.397,-1.817] | 0.616 |  | 36.23 | aligned_error 0.62m > 0.60m; yaw_error 36.2deg > 8.0deg |
