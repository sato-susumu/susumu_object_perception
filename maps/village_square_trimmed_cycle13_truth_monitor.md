# Live SLAM Truth Monitor

- gps_topic: `/TurtleBot3Burger/gps`
- gps_type: `geometry_msgs/msg/PointStamped`
- estimate_frame: `map`
- robot_frame: `base_footprint`
- samples: `170`
- events: `8`
- gps_path_length_m: `16.950`
- estimate_path_length_m: `16.897`
- max_aligned_error_m: `3.607`
- max_heading_error_deg: `55.92`

## Events

| idx | gps rel xy | estimate xy | aligned err | heading err | reason |
|---:|---|---|---:|---:|---|
| 105 | [0.461,-5.048] | [1.077,-4.833] | 0.619 |  | aligned_error 0.62m > 0.60m |
| 114 | [0.478,-4.039] | [1.863,-4.229] | 1.270 | 53.05 | aligned_error 1.27m > 0.60m; heading_error 53.1deg > 30.0deg |
| 122 | [0.938,-3.057] | [2.930,-3.959] | 1.981 | 53.30 | aligned_error 1.98m > 0.60m; heading_error 53.3deg > 30.0deg |
| 132 | [1.591,-2.076] | [4.091,-3.867] | 2.747 | 53.25 | aligned_error 2.75m > 0.60m; heading_error 53.3deg > 30.0deg |
| 140 | [1.982,-1.739] | [4.641,-3.981] | 2.971 |  | aligned_error 2.97m > 0.60m |
| 147 | [2.305,-0.734] | [5.618,-3.623] | 3.603 | 48.72 | aligned_error 3.60m > 0.60m; heading_error 48.7deg > 30.0deg |
| 156 | [2.234,0.430] | [6.497,-2.863] | 3.924 | 41.95 | aligned_error 3.92m > 0.60m; heading_error 41.9deg > 30.0deg |
| 164 | [1.789,1.417] | [7.010,-1.901] | 3.849 | 33.33 | aligned_error 3.85m > 0.60m; heading_error 33.3deg > 30.0deg |
