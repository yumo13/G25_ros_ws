#!/bin/bash

# CAN Interface and GPIO setup service installation script

echo "Installing CAN and GPIO setup service..."

# サービスファイルをsystemdディレクトリにコピー
sudo cp systemd/can-setup.service /etc/systemd/system/

# systemdデーモンをリロード
sudo systemctl daemon-reload

# サービスを有効化（起動時に自動実行）
sudo systemctl enable can-setup.service

echo "CAN setup service installed and enabled."
echo ""
echo "To start service manually:"
echo "  sudo systemctl start can-setup.service"
echo ""
echo "To check status:"
echo "  sudo systemctl status can-setup.service"
echo ""
echo "To view logs:"
echo "  sudo journalctl -u can-setup.service -f"
echo ""
echo "CAN interface and GPIO will be automatically configured on boot."
echo "To run ROS 2 CAN interface node manually:"
echo "  source install/setup.bash"
echo "  ros2 run can_interface can_interface_node"