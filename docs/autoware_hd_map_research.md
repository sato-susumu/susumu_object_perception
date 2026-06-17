# Autoware 全面採用と HD 地図 — 技術調査記録

Autoware（自動運転スタック）を本プロジェクトに全面採用する場合に必要な **HD 地図の作成
コスト**と、**HD 地図が付属するシミュレータ**の実態を調査した記録。多源 Web 調査 +
反証検証（各主張を 3 票の敵対的検証にかけ、2/3 で棄却）で裏取りした結果をまとめる。

> 背景: 現構成は ROS2 Humble + Gazebo Classic / Webots + 自作 perception + Nav2（2D 占有
> 格子のみ）。信号認識は地図無しの自作パイプラインで赤黄青を認識できる段階
> （[`traffic_light_recognition.md`](traffic_light_recognition.md)）。「いっそ Autoware を
> 全面採用してはどうか」を検討するにあたり、最大の懸念だった **Lanelet2 ベクター地図の
> 作成コスト**を中心に調べた。

## 結論（要点）

- **Autoware は 2 種類の地図が必須**: ①点群地図(PCD, NDT 自己位置推定用) ②Lanelet2
  ベクター地図(経路計画・**信号認識**・軌跡予測用)。最大コストは **②の手作業作図**。
- **「地図ゼロで Autoware の信号認識まで動く」のは AWSIM の西新宿サンプルだけ**。任意の
  場所/自分の world では **Lanelet2 を自作する必要があり、ここから逃げられない**。
- **自動化できるのは PCD まで**。Lanelet2（車線・信号 3D 位置・停止線・通行権）は手作業が残る。
- CARLA も OpenDRIVE → Lanelet2 変換に**手修正が必須**（信号要素が自動移行で欠落しやすい）。

## Autoware が要求する地図（確度: 高 / 3-0）

| 地図 | 用途 | 形式 | 作成 |
|---|---|---|---|
| 点群地図 (PCD) | LiDAR 自己位置推定（NDT で生スキャンと事前 PCD を照合） | `.pcd` | LiDAR+SLAM で生成。**自動化可** |
| ベクター地図 (Lanelet2) | 経路計画・**信号認識**・歩行者/車両の軌跡予測 | Lanelet2(`.osm`) | **手作業作図（最大コスト）** |

Lanelet2 が含むべき要素（公式文書ほぼ逐語、確度 高）: 車線の形状・位置、信号、停止線、
横断歩道、駐車枠/駐車場。さらに**各 lanelet が通行権・速度制限・進行方向・関連する信号/
停止線/標識の参照を持つ**必要がある。これが 1 交差点あたりの作図要素の実体。

> GNSS-only / mapless 構成は非標準（コード改変が要る）で、routing/信号/予測のための
> Lanelet2 は依然必要。

## Lanelet2 作図ツール（確度: 高 / 3-0）

| ツール | 種別 | 備考 |
|---|---|---|
| **TIER IV Vector Map Builder (VMB)** | ブラウザ・無料（要 TIER IV アカウント） | **事実上の標準/推奨**。PCD を背景に車線/停止線/信号を手描き → Lanelet2(`.osm`) 出力 |
| **MapToolbox** | Unity プラグイン（autocore-ai, Unity 2019.4+） | OSS 代替。lanelet/信号(参照線+高さ)/停止線/regulatory element/駐車/横断歩道に対応 |
| JOSM | Java | Lanelet2 を作れるが Autoware 互換化に手修正多数 → **公式は非推奨** |

- **信号の登録**: VMB 上で PCD を参照しつつ手描き。信号は **3D 高さ + bulb 参照線(light_bulbs)**
  を設定し、停止線とは **regulatory element で紐付け**る。これで信号位置・色が Lanelet2 に
  符号化され、Autoware の信号認識が機能する。
- 落とし穴: lat/lon を null 化（`remove_lat_lon.py`）しないと車線が無限遠に伸びる。

## HD 地図が付属する/生成しやすいシミュレータ

| 構成 | 地図 | 箱出しで動くか | コスト |
|---|---|---|---|
| **AWSIM 西新宿** | 同梱(PCD+Lanelet2+信号) | ✅ **信号認識まで箱出し** | **ゼロ（ただし西新宿限定）** |
| **CARLA + carla_autoware_bridge** | Town=OpenDRIVE | △ **箱出し不可**。Autoware 用「特別な Lanelet2」へ変換要 | 変換(CommonRoad Scenario Designer)+手修正。純正 Lanelet2 は信号要素欠落・PCD と座標不整合 |
| **OSM→自動 PCD 生成** | OSM から PCD 自動、Lanelet2 は手描き | △ 実証あり | PCD は自動化、**Lanelet2 は手作業**。合成 PCD で実走 NDT 精度未検証・小規模限定 |
| **自前フル作成 (VMB)** | 手作業 | ✅ | **最大** |
| **現状 (Nav2 + 自作 perception)** | 2D 占有格子のみ | ✅ 信号認識は自作で赤黄青動作済み | **地図作成不要** |

### AWSIM 西新宿（確度: 高 / 3-0）
- TIER IV 製 Autoware 公式シミュレータ。`nishishinjuku_autoware_map.zip`(v1.1.0, ~58MB) が
  GitHub リリースで配布され、**PCD + OSM(Lanelet2) + 信号**を含む。ユーザーの地図作図は不要。
- デモ車両に Traffic Light Camera 搭載、**信号認識まで動く**。
- ⚠️ ただしこれは「ショーケース」。**西新宿以外では地図作成が必要**になる（＝任意の場所で
  地図ゼロにはならない）。

### CARLA + Autoware（確度: 高 / 3-0）
- `carla_autoware_bridge`（CARLA 0.9.15 + Autoware Universe Humble）。標準 Town は OpenDRIVE
  で配布され、**Autoware 用 Lanelet2/PCD を箱出しでは含まない**。
- 変換: CommonRoad Scenario Designer 等で OpenDRIVE→Lanelet2、ただし「欠落/誤情報を手作業修正」。
  CARLA 純正 Lanelet2 は信号要素を欠き lat/lon が PCD と不整合 → VMB で手修正が要る。PCD は
  `pcl_recorder` で別生成。**「OpenDRIVE があるから楽」とはならない。**

### OSM→PCD 自動生成（確度: 高 / 3-0、ただし限定的）
- arXiv 2508.16856 + `zubxxr/AV-Map-Creation-Workflow`。OSM2World(OSM→3D メッシュ)→
  CloudCompare(メッシュ→.pcd)→ PCL(向き補正+ASCII→バイナリ) で **PCD を LiDAR/SLAM 無しに
  自動生成**。Lanelet2 はその PCD を背景に VMB で手描き。
- **PCD は自動化できるが Lanelet2 は依然手作業**。合成メッシュ由来 PCD のため実走 NDT 自己
  位置推定の忠実度は未検証、小規模マップ限定（大規模はメモリ/処理で破綻）。

## このプロジェクトへの含意

- **Autoware 全面採用 = Lanelet2 手作業作図が確定コスト**（自分の走行環境で使う場合）。
  PCD は自動化余地があるが、信号・停止線・車線の意味付けは人手が残る。
- つくばチャレンジ勢が Autoware フルでなく **Nav2/2D 格子 + 3D SLAM** を選びがちなのは、
  この Lanelet2 コスト回避が理由（[[tsukuba-challenge-research]] とも整合）。
- 現状の **「Nav2 + 自作信号認識」路線は、この手作業を回避できている**のが利点。
- Autoware を試すなら、まず **AWSIM 西新宿で地図ゼロ評価**するのが低コスト。自分の環境へ
  広げる段で Lanelet2 作成コストが立ち上がる、という段階的判断ができる。

## 未確定事項（今後の調査余地）

- Lanelet2 手作業作図の**定量工数**（1 交差点あたり何時間、1km 市街地あたり何人日）は実数値が
  確認できなかった（作図要素の質的列挙は確実だが工数は未確定）。
- `opendrive2lanelet` / CommonRoad Scenario Designer の **Autoware 互換性・手修正量**の独立評価。
- OSM 由来合成 PCD が**実環境 NDT で実用精度を出せるか**、大規模スケール可否。
- **MORAI** 等 他シミュレータの HD 地図付属状況。**Webots/Gazebo Classic から Autoware 用
  Lanelet2/PCD を生成する確立経路**の有無（現構成からの移行に直結）。

## 出典（一次情報中心）

- Autoware 地図作成 公式: autowarefoundation.github.io/autoware-documentation（creating-maps /
  creating-vector-map/lanelet2）※ URL はドキュメント再編で移動しうる（how-to-guides↔tutorials）
- TIER IV Vector Map Builder: tools.tier4.jp/feature/vector_map_builder_ll2/
- MapToolbox: github.com/autocore-ai/MapToolbox
- AWSIM: autowarefoundation.github.io/AWSIM-Labs（Quick Start Demo）、
  github.com/tier4/AWSIM/releases（nishishinjuku_autoware_map.zip）
- CARLA+Autoware: github.com/HoYongLee98/carla_autoware_bridge、arXiv 2402.11239
- OSM→PCD: arXiv 2508.16856、github.com/zubxxr/av-map-creation-workflow

> 注: 多くの一次 URL は文書再編で 404/301 になりうる（autoware-documentation のパス移動、
> AWSIM の tier4→autowarefoundation 組織移管）。内容は現行 main で同一・存命だが、参照時は
> URL を解決し直すこと。
