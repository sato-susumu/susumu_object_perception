# Waypoint navigation report

- reason: `safe_pose_recovery_failed`
- reached: `11/96`
- missed: `[9, 12]`
- pending: `[13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95]`
- elapsed: `34.395s`
- safe_pose_guard: `True`
- safe_pose_recoveries: `2`
- waypoints: `/home/taro/ros2_ws/src/susumu_object_perception/maps/village_square_trimmed_cycle26_centerline_follow_2m_waypoints.yaml`

| index | result | reason | duration_sec | x | y |
|---:|---|---|---:|---:|---:|
| 0 | reached | succeeded | 2.752 | 1.705 | -1.120 |
| 1 | reached | succeeded | 3.401 | 0.005 | -0.370 |
| 2 | reached | succeeded | 1.201 | -0.495 | 0.130 |
| 3 | reached | succeeded | 0.902 | -0.495 | 0.680 |
| 4 | reached | succeeded | 1.551 | 0.105 | 0.680 |
| 5 | reached | succeeded | 3.301 | 0.105 | 2.680 |
| 6 | reached | succeeded | 2.651 | -0.645 | 4.330 |
| 7 | reached | succeeded | 0.301 | -0.695 | 4.380 |
| 8 | reached | succeeded | 1.101 | -0.995 | 4.680 |
| 9 | missed | safe_pose_cost_99_safe_recovered | 3.501 | 0.605 | 6.380 |
| 10 | reached | succeeded | 1.201 | 1.105 | 6.880 |
| 11 | reached | succeeded | 3.001 | 1.905 | 8.580 |
| 12 | missed | safe_pose_cost_100_safe_pose_timeout | 7.767 | 2.255 | 10.430 |

## Safe Pose Recoveries

| waypoint | trigger | result | reason | duration_sec | safe_x | safe_y |
|---:|---|---|---|---:|---:|---:|
| 9 | safe_pose_cost_99 | succeeded | safe_pose_succeeded | 0.561 | -0.562 | 5.25 |
| 12 | safe_pose_cost_100 | failed | safe_pose_timeout | 5.69 | 1.369 | 7.839 |
