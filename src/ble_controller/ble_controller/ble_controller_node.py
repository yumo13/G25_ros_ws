#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json
import os
import threading
import time

# パイプ通信設定
PIPE_PATH = '/tmp/ble_ros_pipe'

class BleControllerNode(Node):
    """
    パイプ通信でNode.js BLEサーバーからデータを受信し、TwistメッセージとしてパブリッシュするROS2ノード
    """
    def __init__(self):
        super().__init__('ble_controller_node')
        
        # cmd_velトピックへのパブリッシャー作成
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # パイプリーダーの状態管理
        self.pipe_thread = None
        self.running = True
        
        self.get_logger().info('BLE Controller Node initialized')
        self.get_logger().info('Publishing to /cmd_vel topic')
        self.get_logger().info(f'Waiting for data from pipe: {PIPE_PATH}')

    def process_ble_data(self, linear_y, angular_z):
        """
        BLEから受信したデータを処理してTwistメッセージをパブリッシュ
        """
        try:
            # Twistメッセージの作成と設定
            twist_msg = Twist()
            twist_msg.linear.y = float(linear_y)
            twist_msg.angular.z = float(angular_z)
            
            # メッセージをパブリッシュ
            self.publisher_.publish(twist_msg)
            self.get_logger().info(f'Published Twist: linear.y={twist_msg.linear.y}, angular.z={twist_msg.angular.z}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing BLE data: {e}')

    def pipe_reader_thread(self):
        """
        パイプからデータを読み取るスレッド
        """
        self.get_logger().info('Starting pipe reader thread')
        
        while self.running:
            try:
                # パイプが存在するまで待機
                if not os.path.exists(PIPE_PATH):
                    self.get_logger().info(f'Waiting for pipe: {PIPE_PATH}')
                    time.sleep(0.5)
                    continue
                
                self.get_logger().info('Pipe found, opening for reading...')
                
                # パイプを開いてデータを読み取り（ノンブロッキングモード）
                try:
                    pipe_fd = os.open(PIPE_PATH, os.O_RDONLY | os.O_NONBLOCK)
                    pipe_file = os.fdopen(pipe_fd, 'r')
                    
                    self.get_logger().info('Pipe opened successfully, waiting for data...')
                    
                    while self.running:
                        try:
                            line = pipe_file.readline()
                            if line:
                                line = line.strip()
                                if line:
                                    # JSONデータをパース
                                    data = json.loads(line)
                                    linear_y = data.get('linear_y', 0.0)
                                    angular_z = data.get('angular_z', 0.0)
                                    timestamp = data.get('timestamp', 0)
                                    
                                    self.get_logger().info(f'Received from BLE: linear.y={linear_y}, angular.z={angular_z}, ts={timestamp}')
                                    
                                    # データを処理
                                    self.process_ble_data(linear_y, angular_z)
                            else:
                                # データがない場合、短時間待機
                                time.sleep(0.01)
                                
                        except json.JSONDecodeError as e:
                            self.get_logger().error(f'JSON decode error: {e}, line: {line}')
                        except OSError as e:
                            if e.errno == 11:  # EAGAIN - no data available
                                time.sleep(0.01)
                                continue
                            else:
                                self.get_logger().error(f'OS error reading pipe: {e}')
                                break
                        except Exception as e:
                            self.get_logger().error(f'Error reading from pipe: {e}')
                            time.sleep(0.1)
                    
                    pipe_file.close()
                    
                except Exception as e:
                    self.get_logger().error(f'Error opening pipe: {e}')
                    time.sleep(1.0)
                    
            except Exception as e:
                self.get_logger().error(f'Pipe reader thread error: {e}')
                time.sleep(1.0)
        
        self.get_logger().info('Pipe reader thread stopped')

    def start_pipe_reader(self):
        """
        パイプリーダースレッドを開始
        """
        if self.pipe_thread is None or not self.pipe_thread.is_alive():
            self.pipe_thread = threading.Thread(target=self.pipe_reader_thread, daemon=True)
            self.pipe_thread.start()
            self.get_logger().info('Pipe reader thread started')

    def stop_pipe_reader(self):
        """
        パイプリーダースレッドを停止
        """
        self.running = False
        if self.pipe_thread and self.pipe_thread.is_alive():
            self.pipe_thread.join(timeout=2.0)
            self.get_logger().info('Pipe reader thread stopped')

def main(args=None):
    """メイン関数"""
    # ROS2初期化
    rclpy.init(args=args)
    
    try:
        # ノード作成
        ble_node = BleControllerNode()
        
        # パイプリーダーを開始
        ble_node.start_pipe_reader()
        
        # ROS2ノードをスピン
        rclpy.spin(ble_node)
        
    except KeyboardInterrupt:
        print('\nShutting down...')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        # 終了処理
        if 'ble_node' in locals():
            ble_node.stop_pipe_reader()
            ble_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
