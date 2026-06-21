# Waypoint navigation report

- reason: `safe_pose_recovery_failed`
- reached: `9/96`
- missed: `[9]`
- pending: `[10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95]`
- elapsed: `29.274s`
- safe_pose_guard: `True`
- safe_pose_recoveries: `1`
- waypoints: `/home/taro/ros2_ws/src/susumu_object_perception/maps/village_square_trimmed_cycle26_centerline_follow_2m_waypoints.yaml`

| index | result | reason | duration_sec | x | y |
|---:|---|---|---:|---:|---:|
| 0 | reached | succeeded | 2.902 | 1.705 | -1.120 |
| 1 | reached | succeeded | 6.252 | 0.005 | -0.370 |
| 2 | reached | succeeded | 1.451 | -0.495 | 0.130 |
| 3 | reached | succeeded | 0.901 | -0.495 | 0.680 |
| 4 | reached | succeeded | 1.551 | 0.105 | 0.680 |
| 5 | reached | succeeded | 3.402 | 0.105 | 2.680 |
| 6 | reached | succeeded | 2.601 | -0.645 | 4.330 |
| 7 | reached | succeeded | 0.401 | -0.695 | 4.380 |
| 8 | reached | succeeded | 1.001 | -0.995 | 4.680 |
| 9 | missed | safe_pose_cost_99_safe_pose_timeout | 7.298 | 0.605 | 6.380 |

## Safe Pose Recoveries

| waypoint | trigger | result | reason | duration_sec | safe_x | safe_y |
|---:|---|---|---|---:|---:|---:|
| 9 | safe_pose_cost_99 | failed | safe_pose_timeout | 4.774 | -0.767 | 5.112 |
