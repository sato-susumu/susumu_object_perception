# Recognition World Comparison

## Inputs

- world: `webots_worlds/indoor.wbt`
- map: `maps/indoor.yaml`
- db: `/tmp/indoor_object_memory_pruned.sqlite3`
- robot_world_xy: [6.36, 0.0]
- min_existence: 0.5
- min_hits: 2
- ignored_types: ['Sofa', 'Table']
- match_distance_m: 1.0

## Summary

| metric | value |
|---|---:|
| expected_count | 7 |
| detection_count | 4 |
| correct_count | 4 |
| wrong_label_count | 0 |
| missed_without_near_detection_count | 3 |
| extra_detection_count | 0 |
| class_aware_false_positive_count | 0 |
| class_aware_false_negative_count | 3 |
| precision | 1.000 |
| recall | 0.571 |
| f1 | 0.727 |

## Correct Matches

| expected | accepted | detection | det_label | dist_m | exist | hits |
|---|---|---|---|---:|---:|---:|
| PottedTree[3] PottedTree `PottedTree` (2.81, -1.70) | potted plant, vase | #23 (2.66, -1.48) | potted plant | 0.26 | 1.00 | 19 |
| PottedTree[1] PottedTree `potted tree(5)` (0.64, -4.00) | potted plant, vase | #2 (1.11, -3.71) | potted plant | 0.55 | 1.00 | 10 |
| Armchair[1] Armchair `Armchair` (1.26, 1.03) | chair | #18 (1.25, 1.67) | chair | 0.64 | 0.51 | 2 |
| PottedTree[2] PottedTree `potted tree(8)` (-0.56, 3.30) | potted plant, vase | #1 (0.09, 3.05) | potted plant | 0.70 | 1.00 | 16 |

## Wrong Label Near Ground Truth

None.

## Missed Ground Truth

| expected | accepted | map_xy | world_xy |
|---|---|---:|---:|
| Fridge[1] Fridge `Fridge` | refrigerator, fridge | (-0.66, 4.64) | (5.70, 4.64) |
| PottedTree[4] PottedTree `potted tree(3)` | potted plant, vase | (-0.76, -4.50) | (5.60, -4.50) |
| BunchOfSunFlowers[1] BunchOfSunFlowers `BunchOfSunFlowers` | potted plant, vase | (1.32, -0.52) | (7.68, -0.52) |

## Extra Detections

None.

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
| Sofa | `Sofa` | (9.00, -0.52) | ignored by --ignore-type |
| LandscapePainting | `LandscapePainting` | (8.50, -4.98) | default COCO YOLO weights have no painting class |
| Table | `Table` | (7.70, -0.52) | ignored by --ignore-type |
| DirectionPanel | `DirectionPanel` | (8.00, 5.00) | not evaluated by object_memory static object DB |
