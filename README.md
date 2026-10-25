# rabbits_description

[![CI](https://github.com/rabbits-robotics/rabbits_description/actions/workflows/ci.yml/badge.svg)](https://github.com/rabbits-robotics/rabbits_description/actions/workflows/ci.yml)

機体の URDF（xacro）と rviz での表示 launch。`urdf/<機体>/` と `meshes/<機体>/` に機体ごとに置き、機体をまたいで使うもの（色の定義など）だけ `urdf/common/` に置く。

| 機体 | 最上位 | 内容 |
|---|---|---|
| proto1 | `urdf/proto1/proto1.xacro` | オムニ 4 輪 + 砲塔 Yaw / Pitch（メッシュあり） |
| gamma | `urdf/gamma/gamma.xacro` | 脚 4 本（4 節リンク、各脚の先に車輪）+ 首 Yaw / Pitch / Roll + カメラ。**脚は実寸**（`config/gamma_linkage.yaml`）、**首と胴体は仮**（箱と円筒）。メッシュは下の約束で書き出したら差し替える |

```bash
ros2 launch rabbits_description display.launch.py robot:=gamma use_gui:=true   # スライダで関節を動かして確認
ros2 launch rabbits_description display.launch.py robot:=gamma                # joint_states は外（robot_bridge など）から
```

```
urdf/
  common/materials.xacro     色の定義
  proto1/proto1.xacro        最上位 + 部品マクロ（mobile_base, wheel, turret_base, turret_unit）
  gamma/gamma.xacro          最上位 + 部品マクロ（body, leg, neck）
config/gamma_linkage.yaml    gamma の脚リンクと首の寸法（xacro とノードが読む。gamma_turret_fw の robot.hpp と対で更新）
src/linkage_state_publisher.cpp  gamma の派生関節を足すミキサー（motor_joint_states → joint_states）
meshes/
  proto1/*.dae
  gamma/                     メッシュを書き出したら
rviz/display.rviz
```

## gamma

- 関節名は [rabbits_bridge](https://github.com/rabbits-robotics/rabbits_bridge) の `config/gamma.yaml` の `joints`（関節番号順）と同じ: `yaw_joint`, `pitch_joint`, `roll_joint`, `right_front_wheel_joint`, `left_front_wheel_joint`, `right_back_wheel_joint`, `left_back_wheel_joint`, `right_front_leg_joint`, `left_front_leg_joint`, `right_back_leg_joint`, `left_back_leg_joint`。ファームの `JOINT_STATE` → bridge の `joint_states` がそのまま入る。
- 軸の向き（確定。設計書は rabbits_sim の `docs/superpowers/specs/2026-10-24-gamma-sim-design.md` 5 章）: 車輪は車輪リンクの z 軸が回転軸で z 正が機体の外側。脚は角度 0 が最下点（メカスト、原点）で、角度が増えると腕の先の車輪が下がり車体が上がる。首は yaw（z）→ pitch（y、−π/12〜π/6）→ roll（x、±π/6）→ `camera_link`（カメラ）→ `imu_link`（固定、BNO055。bridge の `imu/data` の `frame_id`）。符号は REP-103。
- `imu_link` の向きは `camera_link` と同じ（回転なし）。軸の入れ替えはファームが BNO055 のレジスタで済ませて `imu/data` をカメラ座標系で出すので、TF で二重に回さない。位置は仮（カメラの後ろ 0.03 m）。
- 脚は 4 節リンク（モータ軸 → クランク → 連結リンク → 車輪ホルダ、ホルダを平行に保つバー 2 本。記号と形は [docs/gamma_leg_linkage.svg](docs/gamma_leg_linkage.svg)、設計書は rabcl の `docs/superpowers/specs/2026-10-25-gamma-leg-linkage-design.md`）。URDF は木構造なのでループを連結リンクの先端で切り、各脚に `*_leg_joint`（モータ角、`joint_states`）、`*_bar_joint` と `*_coupler_joint`（派生）、`*_bar_lower_joint` と `*_holder_joint`（`mimic`）、`*_wheel_joint` を置く。派生関節は線形でないので `mimic` では表せず、**`linkage_state_publisher`（ミキサー）** が `motor_joint_states`（bridge / 偽ファームの出力をリマップしたもの）を受け、そのまま写したうえで派生 8 関節を rabcl `LegLinkage` で計算して足し、`joint_states` を 1 本で出す。`/joint_states` の発行元はこのノードだけ（1 トピック 1 発行元）。ミキサーが落ちると `robot_state_publisher` に何も届かない。
- 関節の回転軸はすべて +y（正 = 下向き）。ミキサーが出す値は URDF の向き（bar = −φ、coupler = −(ψ − α)）。
- base 原点は 4 本の脚モータ軸を含む水平面と Yaw 軸の交点（x 前、y 左、z 上）。首 Yaw 原点はその 25.5 mm 下、Pitch / Roll はそこから 126.4 mm 上の同じ点（仮）。
- pitch / roll は論理関節（ファームが差動リンクの 2 モータ角から求めた値）。ロール軸を後回しにする間は 0 が来て静止する。
- rviz では `base_link` が固定なので、車高を上げると車輪が下へ伸びて見える（床は描かない）。
- 車輪の回転の向きは、`joint_states` に入る角度（ファーム / 偽ファームの `OmniDrive × WHEEL_SIGN`）をそのまま表す。現状の `WHEEL_SIGN = -1` では前進指令で後ろに転がって見える（rabbits_sim の README「rviz での車輪の向きの読み方」）。

### gamma のメッシュの座標系（CAD で STL を書き出すときの約束）

STL はリンクごとに 1 ファイル（`meshes/gamma/<リンク>.stl`、バイナリ、mm、粗い解像度で 1 ファイル 1〜2 MB 以下）。**各 STL の原点と軸は、そのリンクの URDF フレームに一致させる**（xacro 側の visual origin を 0 にしておくため）。すべて右手系、ゼロ姿勢（脚はモータ原点 = 最下点、首はまっすぐ前）で書き出す。同じ形の脚と車輪は右前の 1 組だけ書き出し、4 本に使い回す。

| STL | 中身 | 原点 | 軸 |
|---|---|---|---|
| `base.stl` | 胴体と胴体に固定の部品 | 4 本の脚の回転軸を含む水平面と、首 Yaw 軸（機体の鉛直中心線）の交点 | x 前、y 左、z 上 |
| `leg_arm.stl` | 右前の腕（脚モータの出力側で回る部品。平行リンクの 1 本目） | 脚の回転軸上で、腕の幅の中央 | x = 水平で半径方向の外向き（胴体中心からその脚の軸を通る向き）、z 上、y = z × x（= 回転軸）。ゼロ姿勢では腕は x から θ0 だけ下を向いて写る |
| `leg_arm2.stl` | 平行リンクの 2 本目（あれば） | 2 本目の回転軸上（`leg_arm` と同じ向き） | 同上 |
| `knuckle.stl` | 腕の先で車輪を支える部品（車輪モータの固定側を含む） | ナックルの回転軸（腕の先の軸）上で、幅の中央 | `leg_arm` と同じ向き（ナックルは胴体と平行を保つので、ゼロ姿勢でも傾かない） |
| `wheel.stl` | 右前の車輪 | 車輪の中心 | z = 回転軸で z 正が機体の外側（半径方向）。x / y は任意 |
| `neck_yaw.stl` | Yaw 軸で回る部品 | Yaw 軸上の 1 点（モータの出力面など。`base` 原点からの高さを控える） | x 前、y 左、z 上 |
| `neck_pitch.stl` | Pitch 軸で回る部品 | Pitch 軸（y 方向）と機体の中心面（y = 0）の交点 | 同上 |
| `neck_camera.stl` | Roll 軸より先（カメラ、gamma_turret 基板） | Roll 軸（x 方向）上の 1 点（ロールモータの出力面など） | 同上 |

あわせて控える値（mm / deg、各リンクの原点からの相対）:

| 値 | 使う先 |
|---|---|
| 脚の取り付け半径 r（`base` 原点から脚の回転軸までの水平距離） | 脚の関節の位置、`BodyPosture` |
| 腕の長さ L（脚の軸からナックルの軸まで）と、ゼロ姿勢で腕が水平から下がっている角度 θ0、可動範囲（原点から何度上がるか） | ナックルの位置、関節の limit、`BodyPosture` の幾何 |
| 2 本目の腕の軸の位置（1 本目の軸からの xyz、`leg_arm` の向きで） | 平行四辺形 |
| 車輪の中心（ナックル原点からの xyz） | 車輪の関節 |
| Yaw 軸の原点の高さ（`base` 原点から）、Pitch 軸の原点（Yaw 原点から）、Roll 軸の原点（Pitch 原点から） | 首の関節 |
| カメラのレンズ中心と IMU（`neck_camera` 原点から） | `camera_link` / `imu_link` |
| 胴体の外形（当たり判定用の箱の寸法。無くてもよい） | collision |
