#!/usr/bin/env python3
"""
ジャーク軌道コントローラの出力を可視化するスクリプト
軌道完了後にすべてのデータをまとめてプロット
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
import matplotlib.pyplot as plt

try:
    import japanize_matplotlib
except ImportError:
    pass

class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')
        
        # データ保存用リスト（完了後にプロット）
        self.position_time = []
        self.position_data = []
        self.velocity_time = []
        self.velocity_data = []
        self.acceleration_time = []
        self.acceleration_data = []
        
        self.start_time = None
        self.trajectory_completed = False
        self.data_count = 0
        
        self.pos_sub = self.create_subscription(
            Float64,
            'current_position',
            self.position_callback,
            100
        )
        
        self.vel_sub = self.create_subscription(
            Float64,
            'current_velocity',
            self.velocity_callback,
            100
        )
        
        self.acc_sub = self.create_subscription(
            Float64,
            'current_acceleration',
            self.acceleration_callback,
            100
        )
        
        self.completed_sub = self.create_subscription(
            Bool,
            'trajectory_completed',
            self.completed_callback,
            10
        )
        
        self.get_logger().info('Visualizer started. Waiting for trajectory data...')
        
    def get_elapsed_time(self):
        if self.start_time is None:
            self.start_time = self.get_clock().now()
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
    def position_callback(self, msg):
        if not self.trajectory_completed:
            t = self.get_elapsed_time()
            self.position_time.append(t)
            self.position_data.append(msg.data)
        
    def velocity_callback(self, msg):
        if not self.trajectory_completed:
            t = self.get_elapsed_time()
            self.velocity_time.append(t)
            self.velocity_data.append(msg.data)
        
    def acceleration_callback(self, msg):
        if not self.trajectory_completed:
            t = self.get_elapsed_time()
            self.acceleration_time.append(t)
            self.acceleration_data.append(msg.data)
            self.data_count += 1
            
            # 進捗表示
            if self.data_count % 50 == 0:
                self.get_logger().info(f'データ受信中... ({self.data_count} points)')
    
    def completed_callback(self, msg):
        if msg.data and not self.trajectory_completed:
            self.trajectory_completed = True
            self.get_logger().info(f'軌道完了！データをプロットします... (total: {self.data_count} points)')
            self.plot_trajectory()
    
    def plot_trajectory(self):
        """すべてのデータをまとめてプロット"""
        if len(self.position_data) == 0:
            self.get_logger().warn('データがありません')
            return
        
        # プロット作成
        fig, axes = plt.subplots(3, 1, figsize=(12, 9))
        fig.suptitle('Jerk Trajectory Controller - Final Result', fontsize=14, fontweight='bold')
        
        # 位置
        axes[0].plot(self.position_time, self.position_data, 'b-', linewidth=2, label='Position')
        axes[0].set_xlabel('Time [s]')
        axes[0].set_ylabel('Position [m]')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()
        
        # 速度
        axes[1].plot(self.velocity_time, self.velocity_data, 'g-', linewidth=2, label='Velocity')
        axes[1].set_xlabel('Time [s]')
        axes[1].set_ylabel('Velocity [m/s]')
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
        
        # 加速度
        axes[2].plot(self.acceleration_time, self.acceleration_data, 'r-', linewidth=2, label='Acceleration')
        axes[2].set_xlabel('Time [s]')
        axes[2].set_ylabel('Acceleration [m/s²]')
        axes[2].grid(True, alpha=0.3)
        axes[2].legend()
        
        plt.tight_layout()
        
        # 統計情報
        self.get_logger().info(f'プロット完了:')
        self.get_logger().info(f'  位置データ: {len(self.position_data)} points')
        self.get_logger().info(f'  速度データ: {len(self.velocity_data)} points')
        self.get_logger().info(f'  加速度データ: {len(self.acceleration_data)} points')
        if len(self.position_data) > 0:
            self.get_logger().info(f'  最終位置: {self.position_data[-1]:.6f} m')
            self.get_logger().info(f'  総時間: {self.position_time[-1]:.3f} s')
        
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    
    visualizer = TrajectoryVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        print('\n可視化を終了します')
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
