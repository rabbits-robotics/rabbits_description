# rabbits_description

[![CI](https://github.com/rabbits-robotics/rabbits_description/actions/workflows/ci.yml/badge.svg)](https://github.com/rabbits-robotics/rabbits_description/actions/workflows/ci.yml)

機体の URDF（xacro）と rviz での表示 launch。`urdf/<機体>/` と `meshes/<機体>/` に機体ごとに置き、機体をまたいで使うもの（色の定義など）だけ `urdf/common/` に置く。

| 機体 | 最上位 | 内容 |
|---|---|---|
| proto1 | `urdf/proto1/proto1.xacro` | オムニ 4 輪 + 砲塔 Yaw / Pitch（メッシュあり） |
| gamma | `urdf/gamma/gamma.xacro` | 脚 4 本（各脚の先に車輪）+ 首 Yaw / Pitch / Roll + カメラ。**寸法は仮**（箱と円筒）。CAD 確認後にメッシュと寸法を差し替える |

```bash
ros2 launch rabbits_description display.launch.py robot:=gamma use_gui:=true   # スライダで関節を動かして確認
ros2 launch rabbits_description display.launch.py robot:=gamma                # joint_states は外（robot_bridge など）から
```

```
urdf/
  common/materials.xacro     色の定義
  proto1/proto1.xacro        最上位 + 部品マクロ（mobile_base, wheel, turret_base, turret_unit）
  gamma/gamma.xacro          最上位 + 部品マクロ（body, leg, neck）
meshes/
  proto1/*.dae
  gamma/                     CAD 確認後
rviz/display.rviz
```

## gamma

- 関節名は [rabbits_bridge](https://github.com/rabbits-robotics/rabbits_bridge) の `config/gamma.yaml` の `joints`（関節番号順）と同じ: `yaw_joint`, `pitch_joint`, `roll_joint`, `right_front_wheel_joint`, `left_front_wheel_joint`, `right_back_wheel_joint`, `left_back_wheel_joint`, `right_front_leg_joint`, `left_front_leg_joint`, `right_back_leg_joint`, `left_back_leg_joint`。ファームの `JOINT_STATE` → bridge の `joint_states` がそのまま入る。
- 軸の向き（確定。設計書は rabbits_sim の `docs/superpowers/specs/2026-10-24-gamma-sim-design.md` 5 章）: 車輪は車輪リンクの z 軸が回転軸で z 正が機体の外側。脚は角度 0 が最下点（メカスト、原点）で、角度が増えると腕の先の車輪が下がり車体が上がる。首は yaw（z）→ pitch（y、−π/12〜π/6）→ roll（x、±π/6）→ `camera_link`（カメラ）→ `imu_link`（固定、BNO055。bridge の `imu/data` の `frame_id`）。符号は REP-103。
- `imu_link` の向きは `camera_link` と同じ（回転なし）。軸の入れ替えはファームが BNO055 のレジスタで済ませて `imu/data` をカメラ座標系で出すので、TF で二重に回さない。位置は仮（カメラの後ろ 0.03 m）。
- 脚の先に `*_knuckle_joint`（`mimic` で脚と逆向きに −1 倍）を置き、車輪の軸が胴体と平行のまま上下する平行リンクの振る舞いを表している。`joint_states` には載せず、`robot_state_publisher` が脚の角度から計算する。本物は平行リンクなので、CAD で腕の長さ・原点の角度・半径が分かったら 2 本目の腕を足して平行四辺形にする。
- pitch / roll は論理関節（ファームが差動リンクの 2 モータ角から求めた値）。ロール軸を後回しにする間は 0 が来て静止する。
- rviz では `base_link` が固定なので、車高を上げると車輪が下へ伸びて見える（床は描かない）。
- 車輪の回転の向きは、`joint_states` に入る角度（ファーム / 偽ファームの `OmniDrive × WHEEL_SIGN`）をそのまま表す。現状の `WHEEL_SIGN = -1` では前進指令で後ろに転がって見える（rabbits_sim の README「rviz での車輪の向きの読み方」）。
