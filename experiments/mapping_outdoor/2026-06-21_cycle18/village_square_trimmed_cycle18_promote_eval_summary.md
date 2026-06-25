# GLIM 2D map variant evaluation

- cloud: `/home/taro/ros2_ws/src/susumu_object_perception/maps/glim/village_square_trimmed_cycle17_live_topic.ply`
- wbt: `/home/taro/ros2_ws/src/susumu_object_perception/webots_worlds/village_square_trimmed.wbt`
- selected: `topic_pose`
- adopted map: `/home/taro/ros2_ws/src/susumu_object_perception/maps/village_square_trimmed_cycle18_promoted_glim2d.yaml`
- waypoints: `/home/taro/ros2_ws/src/susumu_object_perception/maps/village_square_trimmed_cycle18_promoted_glim2d_waypoints.yaml`
- rule: choose the lowest `unknown_cells` variant that keeps `near_ratio_inside` and `fence_mean_coverage` within 0.020 of the no-trajectory baseline.

| label | adopted | status | rays | unknown | near | fence cov | fence free/unk/occ |
|---|---:|---|---:|---:|---:|---:|---|
| `none` |  | ok | 0 | 599289 | 0.779661 | 0.807602 | 0/38/64 |
| `topic_pose` | yes | ok | 122558 | 423207 | 0.779661 | 0.807602 | 8/30/64 |

Per-variant PNG/JSON/CSV paths are recorded in the summary JSON/CSV.
