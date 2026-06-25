# Recognition World Comparison

## Inputs

- world: `/home/taro/ros2_ws/src/susumu_object_perception/webots_worlds/indoor.wbt`
- map: `/home/taro/ros2_ws/src/susumu_object_perception/outputs/mapping_indoor/indoor.yaml`
- db: `/home/taro/.ros/object_memory.sqlite3`
- robot_world_xy: [6.36, 0.0]
- min_existence: 0.5
- min_hits: 2
- ignored_types: []
- match_distance_m: 1.0
- expected_map_support_dist_m: 0.55

## Summary

| metric | value |
|---|---:|
| expected_count | 9 |
| detection_count | 4 |
| correct_count | 3 |
| wrong_label_count | 0 |
| missed_count | 6 |
| missed_with_near_detection_count | 0 |
| missed_without_near_detection_count | 6 |
| missed_with_near_label_detection_count | 0 |
| expected_with_map_support_count | 9 |
| missed_with_map_support_count | 6 |
| extra_detection_count | 1 |
| class_aware_false_positive_count | 1 |
| class_aware_false_negative_count | 6 |
| precision | 0.750 |
| recall | 0.333 |
| f1 | 0.462 |

## Correct Matches

| expected | accepted | map_occ_m | detection | det_label | dist_m | exist | hits |
|---|---|---:|---|---|---:|---:|---:|
| PottedTree[3] PottedTree `PottedTree` (2.81, -1.70) | potted plant, vase | 0.04 | #3 (3.37, -1.47) | potted plant | 0.60 | 0.90 | 3 |
| PottedTree[2] PottedTree `potted tree(8)` (-0.56, 3.30) | potted plant, vase | 0.06 | #4 (0.29, 3.33) | potted plant | 0.85 | 0.80 | 11 |
| Sofa[1] Sofa `Sofa` (2.64, -0.52) | couch, sofa | 0.25 | #5 (3.48, -0.87) | couch | 0.91 | 0.51 | 2 |

## Wrong Label Near Ground Truth

None.

## Missed Ground Truth

| expected | accepted | map_xy | world_xy | map_occ_m | map_support | nearest_any | nearest_label |
|---|---|---:|---:|---:|---|---|---|
| PottedTree[1] PottedTree `potted tree(5)` | potted plant, vase | (0.64, -4.00) | (7.00, -4.00) | 0.06 | True | #3 potted plant 3.72m label-ok | #3 potted plant 3.72m label-ok |
| Fridge[1] Fridge `Fridge` | refrigerator, fridge | (-0.66, 4.64) | (5.70, 4.64) | 0.32 | True | #4 potted plant 1.62m |  |
| Armchair[1] Armchair `Armchair` | chair | (1.26, 1.03) | (7.62, 1.03) | 0.15 | True | #6 dining table 1.73m |  |
| PottedTree[4] PottedTree `potted tree(3)` | potted plant, vase | (-0.76, -4.50) | (5.60, -4.50) | 0.16 | True | #3 potted plant 5.12m label-ok | #3 potted plant 5.12m label-ok |
| Table[1] Table `Table` | dining table, table | (1.34, -0.52) | (7.70, -0.52) | 0.06 | True | #5 couch 2.17m | #6 dining table 2.37m label-ok |
| BunchOfSunFlowers[1] BunchOfSunFlowers `BunchOfSunFlowers` | potted plant, vase | (1.32, -0.52) | (7.68, -0.52) | 0.04 | True | #5 couch 2.19m | #3 potted plant 2.26m label-ok |

## Extra Detections

| detection | label | map_xy | exist | hits |
|---|---|---:|---:|---:|
| #6 | dining table | (2.98, 1.19) | 1.00 | 12 |

## Skipped World Objects

| type | name | world_xy | reason |
|---|---|---:|---|
| Floor | `floor(1)` | (7.50, 0.00) | structural element |
| Window | `window(1)` | (6.50, -5.10) | structural element |
| Wall | `wall(5)` | (7.50, 5.10) | structural element |
| Wall | `wall(7)` | (5.50, -5.10) | structural element |
| Wall | `wall(10)` | (8.50, -5.10) | structural element |
| Wall | `wall(6)` | (10.10, 0.00) | structural element |
| Wall | `wall(2)` | (5.00, 2.00) | structural element |
| Wall | `wall(1)` | (5.00, -3.50) | structural element |
| Radiator | `Radiator` | (7.33, 4.87) | default COCO YOLO weights have no radiator class |
| Door | `Door` | (5.00, -1.50) | structural element, not an object recognition target |
| Cabinet | `Cabinet` | (9.50, -2.50) | default COCO YOLO weights have no cabinet class |
| CardboardBox | `CardboardBox` | (9.70, 4.70) | default COCO YOLO weights have no cardboard box class |
| CardboardBox | `cardboard box(1)` | (9.00, 4.60) | default COCO YOLO weights have no cardboard box class |
| CardboardBox | `cardboard box(2)` | (9.60, 4.00) | default COCO YOLO weights have no cardboard box class |
| CardboardBox | `cardboard box(3)` | (9.40, 4.50) | default COCO YOLO weights have no cardboard box class |
| FloorLight | `floor light(1)` | (8.84, 1.00) | default COCO YOLO weights have no floor light/lamp class |
| LandscapePainting | `LandscapePainting` | (8.50, -4.98) | default COCO YOLO weights have no painting class |
| DirectionPanel | `DirectionPanel` | (8.00, 5.00) | not evaluated by object_memory static object DB |
