# ROS 2 CAN Interface

ROS 2を使用してSocketCANとの間でCANフレームを送受信するパッケージです。

## 機能

- CANフレームをROS 2メッセージ（`can_msgs::msg::Frame`）として受信・配信
- ROS 2メッセージをCANフレームとして送信
- systemdサービスによる自動CAN設定とGPIO制御

## 構成

- `src/can_interface/` - メインパッケージ
- `src/can_msgs/` - CANメッセージ定義
- `systemd/` - systemdサービスファイル

## インストール

1. パッケージをビルド:
```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

2. CAN/GPIO設定サービスをインストール:
```bash
./install_services.sh
```

## 使用方法

### CAN/GPIO設定サービスの開始（自動起動設定済み）
```bash
sudo systemctl start can-setup.service
```

### ROS 2ノードの実行
```bash
source install/setup.bash
ros2 run can_interface can_interface_node
```

### CANフレーム送信
```bash
ros2 topic pub /can_tx can_msgs/msg/Frame "{id: 100, is_extended: false, is_remote: false, length: 2, data: [255, 255, 0, 0, 0, 0, 0, 0]}" -1
```

### CANフレーム受信確認
```bash
ros2 topic echo /can_rx
```

## systemdサービス

### can-setup.service
- システム起動時に自動実行
- CANインターフェース（can0）の設定（bitrate 1000000）
- GPIO4をOUTPUT, LOWに設定 (STBYモードの解除)
- 終了時にCANインターフェースをdown

## ログ確認

```bash
# サービスのステータス
sudo systemctl status can-setup.service

# リアルタイムログ
sudo journalctl -u can-setup.service -f
```

## パラメータ

- `interface`: CANインターフェース名（デフォルト: "can0"）

## トピック

- `/can_rx` - 受信CANフレーム（Publisher）
- `/can_tx` - 送信CANフレーム（Subscriber）

## 必要な権限

- CAN操作にはroot権限が必要
- GPIO操作にはroot権限が必要
- systemdサービスとして実行することを推奨




起動方法(ble)
ble_serverで

```
sudo /home/kiks/.nvm/versions/node/v18.20.8/bin/node /home/kiks/workspaces/nav_robot/ble_server/ble_server.js
```

ros2 run ble_controller ble_controller_node