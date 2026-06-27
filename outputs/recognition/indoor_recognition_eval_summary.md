# Recognition Eval Summary

- validation_passed: `true`
- schema_version: `2`
- criteria: `{'min_best_f1': 0.7, 'required_best_label': 'ignore_table_sofa'}`
- best_by_f1: `ignore_table_sofa` (`0.727`)

| label | expected | detections | correct | wrong | extra | precision | recall | F1 | FN | missed_map_support |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| all_targets | 9 | 4 | 3 | 0 | 1 | 0.750 | 0.333 | 0.462 | 6 | 6 |
| ignore_table_sofa | 7 | 4 | 4 | 0 | 0 | 1.000 | 0.571 | 0.727 | 3 | 0 |

## Failure Details

### all_targets

- missed_type_hist: `{'PottedTree': 2, 'Fridge': 1, 'Armchair': 1, 'Table': 1, 'BunchOfSunFlowers': 1}`
- extra_class_hist: `{'dining table': 1}`

| missed | type | labels | map support | nearest class | nearest dist | nearest label dist |
|---|---|---|---|---|---:|---:|
| potted tree(5) | PottedTree | potted plant,vase | True | potted plant | 3.72 | 3.72 |
| Fridge | Fridge | refrigerator,fridge | True | potted plant | 1.62 |  |
| Armchair | Armchair | chair | True | dining table | 1.73 |  |
| potted tree(3) | PottedTree | potted plant,vase | True | potted plant | 5.12 | 5.12 |
| Table | Table | dining table,table | True | couch | 2.17 | 2.37 |
| BunchOfSunFlowers | BunchOfSunFlowers | potted plant,vase | True | couch | 2.19 | 2.26 |

| extra id | class | hits | existence |
|---:|---|---:|---:|
| 6 | dining table | 12 | 0.999 |

### ignore_table_sofa

- missed_type_hist: `{'Fridge': 1, 'PottedTree': 1, 'BunchOfSunFlowers': 1}`
- extra_class_hist: `{}`

| missed | type | labels | map support | nearest class | nearest dist | nearest label dist |
|---|---|---|---|---|---:|---:|
| Fridge | Fridge | refrigerator,fridge | None |  |  |  |
| potted tree(3) | PottedTree | potted plant,vase | None |  |  |  |
| BunchOfSunFlowers | BunchOfSunFlowers | potted plant,vase | None |  |  |  |
