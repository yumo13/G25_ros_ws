# Motor Data Logger

モータの目標速度、実際の速度、電圧をCSVファイルに記録するROS 2パッケージ

## 概要

このノードは以下のデータを収集してCSVファイルに保存します：
- **目標速度**: `cmd_vel`トピックから計算される左右モータの目標速度
- **実際の速度**: CAN ID 0x300で送信される左右モータの実際の速度
- **モータ電圧**: CAN ID 0x301で送信される左右モータの印加電圧

## 機能

- `cmd_vel`を受信すると自動的にログ記録を開始
- タイムスタンプ付きCSVファイルを自動生成
- リアルタイムでデータを記録（50Hzで書き込み）

## ビルド

```bash
colcon build --packages-select motor_data_logger
source install/setup.bash
```

## 実行

### 基本的な使い方

```bash
ros2 run motor_data_logger motor_data_logger_node
```

### パラメータ付きで実行

```bash
ros2 run motor_data_logger motor_data_logger_node \
  --ros-args \
  -p speed_can_id:=0x300 \
  -p voltage_can_id:=0x301 \
  -p auto_start:=false
```

## パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
|------------|-----|------------|------|
| `speed_can_id` | int | 0x300 | 速度データを送信するCAN ID |
| `voltage_can_id` | int | 0x301 | 電圧データを送信するCAN ID |
| `auto_start` | bool | false | 起動時から記録開始 |

## トピック

### Subscribe
- `/motor_commands` (std_msgs/Float32MultiArray): 目標速度
- `/can_rx` (can_msgs/Frame): CANバスからの受信データ

## 出力ファイル

CSVファイルは `motor_data_YYYYMMDD_HHMMSS.csv` という名前で生成されます。

### CSVフォーマット

```csv
timestamp,target_left,target_right,actual_left,actual_right,voltage_left,voltage_right
0.000000,0.500000,-0.500000,0.485000,-0.492000,3.240000,3.210000
0.020000,0.500000,-0.500000,0.498000,-0.501000,3.250000,3.230000
...
```

各列の説明：
- `timestamp`: 記録開始からの経過時間 [s]
- `target_left`: 左モータの目標速度 [m/s]
- `target_right`: 右モータの目標速度 [m/s]
- `actual_left`: 左モータの実際の速度 [m/s]
- `actual_right`: 右モータの実際の速度 [m/s]
- `voltage_left`: 左モータの印加電圧 [V]
- `voltage_right`: 右モータの印加電圧 [V]
