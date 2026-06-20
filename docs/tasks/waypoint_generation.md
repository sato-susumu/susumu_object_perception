# ウェイポイント生成タスク — 保存地図から巡回点列を作る

このページは README のタスク一覧「ウェイポイント生成」の詳細ページ。マッピングタスクで合格した
保存地図から、Nav2 で巡回しやすいウェイポイント YAML と確認用 PNG をオフライン生成する。

## 入出力

| 項目 | 内容 |
|---|---|
| 入力 | `maps/<world>.yaml` と対応する PGM |
| 実行 | `scripts/generate_waypoints.py`（`ros2 run susumu_object_perception generate_waypoints.py`） |
| 出力 | `maps/<world>_waypoints.yaml`、`maps/<world>_waypoints.png` |
| 次タスク | [巡回ナビ](waypoint_navigation.md) |

## 実行

```bash
cd ~/ros2_ws/src/susumu_object_perception
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash

ros2 run susumu_object_perception generate_waypoints.py \
  --map maps/city.yaml \
  --out maps/city_waypoints.yaml \
  --spacing 1.5 \
  --clearance 0.4 \
  --connect-clearance 0.30
```

出力 YAML は map 座標の点列。PNG は地図上にウェイポイント番号、巡回経路、配置可能領域、
連結対象領域を重ねた確認用画像。

## 生成アルゴリズム

`generate_waypoints.py` は「なるべく広く回る」と「Nav2 で完走できる」を両立させるため、次の順で処理する。

1. PGM/YAML を読み、free / occupied / unknown を分類する。
2. occupied と unknown からの距離変換を作る。
3. **連結用 clearance**（既定 `0.30m`）で通れる領域を作り、最大連結成分だけを巡回対象にする。
4. **配置用 clearance**（既定 `0.60m`、例では `0.40m`）を満たすセルだけから、`spacing` グリッドで候補点を作る。
5. 点間距離は直線距離ではなく、連結成分上の**測地距離**を使う。
6. 最近傍法 + 2-opt で巡回順を作り、YAML/PNG を保存する。

連結用と配置用の clearance を分けるのが重要。連結判定まで厳しくすると、ドアや家具の隙間で部屋が
分断される。配置判定は壁から離したいので、連結判定より厳しくする。

## 合格基準

1. **入力地図がマッピングタスク合格済み**
   幾何が崩れた地図からウェイポイントを作らない。地図不良をウェイポイント側で補正しない。

2. **巡回対象が到達可能な最大連結成分に乗っている**
   PNG 上でウェイポイントが壁・unknown・別連結成分に置かれていない。経路線が壁を直線で跨いで見えても、
   測地順としては通行可能領域上の近い点同士になっていること。

3. **地図全体を偏りなくカバーしている**
   到達可能な主要空間に点が配置され、端や別室が抜けていない。点が少なすぎる場合は `spacing` を下げる。
   点が多すぎて巡回が長すぎる場合は `spacing` を上げる。

4. **Nav2 で完走できる点列になっている**
   生成ログの最大測地ジャンプが過大でない。通常は `spacing` 近傍、広くても数 m 程度に収まる。
   壁越しの大ジャンプが出る場合は地図の連結性、`connect-clearance`、`clearance` を見直す。

5. **成果物が追跡可能**
   `<world>_waypoints.yaml` と `<world>_waypoints.png` を同時に残す。PNG はレビュー時に最初に見る成果物。

## 調整指針

| 症状 | 主に見る値 | 対処 |
|---|---|---|
| 点が壁に近く、巡回中にこすりやすい | `--clearance` | 上げる |
| 部屋や通路が別連結成分になり、点が置かれない | `--connect-clearance` | 下げる |
| 点が少なく巡回範囲が粗い | `--spacing` | 下げる |
| 点が多く一周が長すぎる | `--spacing` | 上げる |
| 壁越しのような大ジャンプがある | 地図品質、`--connect-clearance` | 地図を確認し、必要なら再マッピング |

## 関連

- [マッピングタスク](mapping.md)
- [巡回ナビタスク](waypoint_navigation.md)
- [launch 一覧](../launch.md)
