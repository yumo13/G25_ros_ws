# G25 ROS2 Workspace

校内案内ロボットG25用のROS 2ワークスペースです。CAN通信、BLE入力、ジョイスティック変換などを含みます。

## 機能

- CANフレームの送受信（SocketCAN ↔ ROS 2）
- BLE入力をTwistに変換して`/cmd_vel`へ配信
- ジョイスティック入力をTwistに変換
- 速度・位置・加速度の軌道可視化
- systemdサービスによるCAN/GPIO初期化

## ディレクトリ構成

- [ble_server](ble_server) - BLEサーバー（Node.js）
- [src/can_interface](src/can_interface) - CANインターフェースノード
- [src/can_msgs](src/can_msgs) - CANメッセージ定義
- [src/ble_controller](src/ble_controller) - BLE → Twist 変換ノード
- [src/debug_tools/joy_controller](src/debug_tools/joy_controller) - Joy → Twist
- [src/debug_tools/jerk_trajectory_controller](src/debug_tools/jerk_trajectory_controller) - 軌道可視化
- [systemd](systemd) - systemdサービス

## 必要要件

- ROS 2 Humble
- Node.js(BLEサーバー用)
- CAN対応デバイス(Raspberry Pi内蔵CAN、CANable2など)

## ビルド

```bash
colcon build --symlink-install
```

## 依存関係インストール

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y 
```

```bash
cd ble_server
npm install
cd ..
```

また、ble_server実行のため、以下のコマンドを実行してください
```bash
sudo setcap cap_net_raw+eip $(eval readlink -f `which node`)
```

## CAN/GPIO設定
### Raspberry Piの場合

1. サービスをインストール:

```bash
./install_services.sh
```

2. サービス起動（自動起動設定済み）:

```bash
sudo systemctl start can-setup.service
```

3. ログ確認

```bash
sudo systemctl status can-setup.service
sudo journalctl -u can-setup.service -f
```

### 他の環境の場合
CANable2などのデバイスを使用する場合、適宜CANインターフェースを設定してください。
``` bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

参考: [CANable 2.0で高速can通信 with ROS 2](https://qiita.com/kzs321kzs/items/463f1b1ce21877e8b4b6)

## 実行方法

### 環境セットアップ

```bash
source install/setup.bash
```

### CANインターフェース

```bash
ros2 run can_interface can_interface_node
```

CANフレーム送信:

```bash
ros2 topic pub /can_tx can_msgs/msg/Frame "{id: 100, is_extended: false, is_remote: false, length: 2, data: [255, 255, 0, 0, 0, 0, 0, 0]}" -1
```

CANフレーム受信確認:

```bash
ros2 topic echo /can_rx
```

### BLEコントローラ

1. BLEサーバー起動:

```bash
node ble_server/ble_server.js
```

2. ROS 2ノード起動:

```bash
ros2 run ble_controller ble_controller_node
```

### ジョイスティック → Twist

```bash
ros2 run joy_controller joy_to_twist
```

### 軌道可視化

```bash
ros2 run jerk_trajectory_controller visualize_trajectory.py
```

## トピック

- `/can_rx` - 受信CANフレーム（Publisher）
- `/can_tx` - 送信CANフレーム（Subscriber）
- `/cmd_vel` - 速度指令（Publisher）
- `joy` - ジョイスティック入力（Subscriber）

## パラメータ

- `interface`（can_interface）: CANインターフェース名（デフォルト: "can0"）
- `max_linear_speed`（joy_controller）: 直進最大速度
- `max_angular_speed`（joy_controller）: 旋回最大速度

## 注意

- CAN/GPIO操作にはroot権限が必要です。
- BLEサーバー実行にはNode.jsが必要です。