# Nav2 pose/costmap monitor report

- samples: `406`
- events: `28`
- diagnosis_counts: `{'local_missing': 1, 'ok': 304, 'plan_no_free_points_in_local_costmap': 4, 'plan_not_in_local_costmap': 17, 'pose_global_lethal': 35, 'pose_global_lethal_static_free': 37, 'pose_static_lethal': 8}`
- event_counts: `{'local_missing': 1, 'plan_no_free_points_in_local_costmap': 2, 'plan_not_in_local_costmap': 5, 'pose_global_lethal': 7, 'pose_global_lethal_static_free': 10, 'pose_static_lethal': 3}`

| time | waypoint | diagnosis | map x | map y | static | global | local | local plan inside/free | scan front |
|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|
| 61.504 | 10 | pose_global_lethal_static_free | 7.03 | -8.18 | 0 | 99 | 0 | 0/0 | 0.975 |
| 63.505 | 10 | pose_global_lethal_static_free | 7.03 | -8.18 | 0 | 99 | 0 | 0/0 | 0.975 |
| 66.004 | 10 | pose_global_lethal_static_free | 7.016 | -8.194 | 0 | 99 | 0 | 0/0 | 0.971 |
| 68.004 | 10 | pose_static_lethal | 7.008 | -8.499 | 100 | 100 | 0 | 0/0 | 1.038 |
| 70.005 | 10 | pose_static_lethal | 7.008 | -8.499 | 100 | 100 | 0 | 0/0 | 0.996 |
| 72.504 | 11 | pose_global_lethal | 7.662 | -8.691 | -1 | 99 | 0 | 3/3 | 0.96 |
| 74.504 | 11 | pose_global_lethal | 7.662 | -8.691 | -1 | 99 | 0 | 3/3 | 0.96 |
| 77.004 | 11 | pose_global_lethal | 7.682 | -8.694 | -1 | 99 | 70 | 3/3 | 0.96 |
| 105.505 | 13 | pose_global_lethal | 6.958 | -10.377 | -1 | 99 | 61 | 69/54 | 0.396 |
| 110.005 | 14 | pose_global_lethal_static_free | 5.389 | -2.824 | 0 | 91 | 0 | 39/39 | 0.901 |
| 127.504 | 15 | plan_not_in_local_costmap | 4.044 | 4.843 | 0 | 0 | 0 | 0/0 | 1.486 |
| 130.505 | 15 | pose_global_lethal_static_free | 6.641 | 4.182 | 0 | 99 | 0 | 44/44 | 1.286 |
| 133.005 | 16 | pose_global_lethal_static_free | 6.559 | 3.028 | 0 | 99 | 0 | 49/25 | 1.031 |
| 135.504 | 16 | plan_no_free_points_in_local_costmap | 6.464 | 2.632 | 0 | 48 | 0 | 12/0 | 0.954 |
| 137.505 | 16 | plan_not_in_local_costmap | 0.324 | 3.696 | 0 | 0 | 0 | 0/0 | 0.951 |
| 140.004 | 16 | plan_not_in_local_costmap | 0.324 | 3.696 | 0 | 0 | 0 | 0/0 | 0.949 |
| 143.505 | 16 | plan_no_free_points_in_local_costmap | 0.573 | 3.593 | 0 | 0 | 0 | 5/0 | 0.865 |
| 148.006 | 17 | pose_static_lethal | 4.039 | 2.555 | 100 | 99 | 0 | 61/61 | 0.847 |
| 150.505 | 17 | pose_global_lethal_static_free | 3.889 | 1.815 | 0 | 99 | 0 | 35/25 | 0.885 |
| 152.505 | 17 | pose_global_lethal_static_free | 4.098 | 2.553 | 0 | 99 | 0 | 2/0 | 0.813 |
