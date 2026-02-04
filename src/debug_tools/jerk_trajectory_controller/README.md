# ジャーク制御軌道生成パッケージ

## 概要

ジャーク制御を用いてS字軌道（7セグメント軌道）を生成し、滑らかな加速・減速を実現するROS 2パッケージです。

指定した距離を移動する際に、以下の制約を満たす軌道を計算します：
- 最大速度
- 最大加速度
- 最大ジャーク（加速度の変化率）

## 軌道生成方式

### 7セグメント軌道（S字軌道）

加速フェーズと減速フェーズで構成されます：

```
加速フェーズ: Phase 1 → Phase 2 → Phase 3
一定速度フェーズ: Phase 4
減速フェーズ: Phase 5 → Phase 6 → Phase 7
```

各フェーズ：
- **Phase 1, 5**: ジャークフェーズ（加速度上昇）
- **Phase 2, 6**: 一定加速度フェーズ
- **Phase 3, 7**: ジャークフェーズ（加速度低下）
- **Phase 4**: 一定速度フェーズ

## ノード

### jerk_trajectory_node

軌道生成ノード

#### パラメータ

| パラメータ | 型 | デフォルト | 説明 |
|-----------|-----|--------|------|
| target_distance | double | 1.0 | 目標移動距離 [m] |
| max_velocity | double | 0.5 | 最大速度 [m/s] |
| max_acceleration | double | 0.5 | 最大加速度 [m/s²] |
| max_jerk | double | 1.0 | 最大ジャーク [m/s³] |
| control_frequency | double | 50.0 | 制御周波数 [Hz] |
| auto_start | bool | false | 自動開始するか |

#### パブリッシャー

| トピック | 型 | 説明 |
|---------|-----|------|
| cmd_vel | geometry_msgs/Twist | 速度指令 |
| current_position | std_msgs/Float64 | 現在位置 [m] |
| current_velocity | std_msgs/Float64 | 現在速度 [m/s] |
| current_acceleration | std_msgs/Float64 | 現在加速度 [m/s²] |
| trajectory_state | std_msgs/Float64MultiArray | 軌道状態（位置、速度、加速度） |
| trajectory_completed | std_msgs/Bool | 軌道完了フラグ |

#### サブスクライバー

| トピック | 型 | 説明 |
|---------|-----|------|
| start_trajectory | std_msgs/Bool | 軌道開始指令 |

## 使用方法

### ノード実行

```bash
# 基本実行
ros2 run jerk_trajectory_controller jerk_trajectory_node

# パラメータ指定
ros2 run jerk_trajectory_controller jerk_trajectory_node --ros-args \
  -p target_distance:=2.0 \
  -p max_velocity:=1.0 \
  -p max_acceleration:=0.5 \
  -p max_jerk:=1.0
```

### Launch ファイル実行

```bash
ros2 launch jerk_trajectory_controller jerk_trajectory.launch.py \
  target_distance:=2.0 \
  max_velocity:=1.0 \
  max_acceleration:=0.5 \
  max_jerk:=1.0
```

### 軌道開始

```bash
# 軌道生成開始
ros2 topic pub /start_trajectory std_msgs/Bool "data: true" -1
```

### 軌道情報の確認

```bash
# 現在位置確認
ros2 topic echo /current_position

# 現在速度確認
ros2 topic echo /current_velocity

# 完了確認
ros2 topic echo /trajectory_completed
```

## 動作確認用スクリプト

可視化スクリプトが利用可能です：

```bash
python3 src/debug_tools/jerk_trajectory_controller/scripts/visualize_trajectory.py
```

## 理論

### S字軌道パラメータ計算

最大速度に到達するかに応じて２つのプロファイルを選択：

1. **三角形加速度プロファイル**（最大加速度に到達しない）
   - ジャークフェーズのみで目標速度に到達
   - $T_j = \sqrt{v_{max} / j_{max}}$

2. **台形加速度プロファイル**（最大加速度に到達）
   - ジャーク → 一定加速度 → ジャーク
   - $T_j = a_{max} / j_{max}$
   - $T_a = (v_{max} - v_{jerk}) / a_{max}$