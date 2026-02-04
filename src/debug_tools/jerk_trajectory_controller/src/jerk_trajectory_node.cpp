#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <cmath>

/**
 * @brief ジャーク制御を用いてy方向への軌道を生成するノード
 * 
 * 指定した距離、最大速度、最大加速度、最大ジャークの制限を満たしながら
 * y方向への移動を行うcmd_velメッセージを生成します。
 */
class JerkTrajectoryController : public rclcpp::Node {
public:
    JerkTrajectoryController() : Node("jerk_trajectory_controller") {
        // パラメータ宣言
        this->declare_parameter<double>("target_distance", 1.0);     // 目標距離 [m]
        this->declare_parameter<double>("max_velocity", 0.5);        // 最大速度 [m/s]
        this->declare_parameter<double>("max_acceleration", 0.5);    // 最大加速度 [m/s^2]
        this->declare_parameter<double>("max_jerk", 1.0);            // 最大ジャーク [m/s^3]
        this->declare_parameter<double>("control_frequency", 50.0);  // 制御周波数 [Hz]
        this->declare_parameter<bool>("auto_start", false);          // 自動開始
        
        // パラメータ取得
        this->get_parameter("target_distance", target_distance_);
        this->get_parameter("max_velocity", max_velocity_);
        this->get_parameter("max_acceleration", max_acceleration_);
        this->get_parameter("max_jerk", max_jerk_);
        this->get_parameter("control_frequency", control_frequency_);
        this->get_parameter("auto_start", auto_start_);
        
        // Publisher と Subscriber
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        position_pub_ = this->create_publisher<std_msgs::msg::Float64>("current_position", 10);
        velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("current_velocity", 10);
        acceleration_pub_ = this->create_publisher<std_msgs::msg::Float64>("current_acceleration", 10);
        trajectory_state_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("trajectory_state", 10);
        completed_pub_ = this->create_publisher<std_msgs::msg::Bool>("trajectory_completed", 10);
        
        start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "start_trajectory", 10,
            std::bind(&JerkTrajectoryController::startCallback, this, std::placeholders::_1));
        
        // 制御ループタイマー
        dt_ = 1.0 / control_frequency_;
        control_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_),
            std::bind(&JerkTrajectoryController::controlLoop, this));
        
        // 軌道パラメータの計算
        calculateTrajectoryParameters();
        
        // 初期化
        is_running_ = auto_start_;
        trajectory_time_ = 0.0;
        current_position_ = 0.0;
        current_velocity_ = 0.0;
        current_acceleration_ = 0.0;
        
        RCLCPP_INFO(this->get_logger(), 
                    "ジャーク制御軌道生成ノード開始\n"
                    "  目標距離: %.3f m\n"
                    "  最大速度: %.3f m/s\n"
                    "  最大加速度: %.3f m/s^2\n"
                    "  最大ジャーク: %.3f m/s^3\n"
                    "  総時間: %.3f s\n"
                    "  制御周波数: %.1f Hz",
                    target_distance_, max_velocity_, max_acceleration_, max_jerk_,
                    total_time_, control_frequency_);
    }
    
private:
    /**
     * @brief S字軌道（7セグメント）のパラメータを計算
     * 
     * シンプルで正確な実装：
     * - Phase 1,3,5,7: ジャークフェーズ（各 Tj 秒）
     * - Phase 2,6: 一定加速度フェーズ（各 Ta 秒）
     * - Phase 4: 一定速度フェーズ（Tv 秒）
     */
    void calculateTrajectoryParameters() {
        // Step 1: 基本時間の計算
        // 最大加速度に達するのに必要な時間
        Tj_ = max_acceleration_ / max_jerk_;
        
        // ジャークフェーズのみで達成できる速度
        double v_at_max_jerk = max_jerk_ * Tj_ * Tj_;  // = a_max^2 / j_max
        
        // Step 2: 最大速度に達するかチェック
        if (max_velocity_ <= v_at_max_jerk) {
            // 最大加速度に達しない（三角形加速度プロファイル）
            Tj_ = std::sqrt(max_velocity_ / max_jerk_);
            Ta_ = 0.0;
            actual_max_acc_ = max_jerk_ * Tj_;
        } else {
            // 最大加速度に達する（台形加速度プロファイル）
            actual_max_acc_ = max_acceleration_;
            // 一定加速度の時間
            Ta_ = (max_velocity_ - v_at_max_jerk) / max_acceleration_;
        }
        
        actual_max_velocity_ = max_velocity_;
        
        // Step 3: 加速フェーズの距離を計算
        double s_accel = calculatePhaseDistance(Tj_, Ta_, actual_max_acc_);
        
        // Step 4: 一定速度フェーズの計算
        double s_cruise = target_distance_ - 2.0 * s_accel;
        
        if (s_cruise >= 0) {
            // 最大速度に達する
            Tv_ = s_cruise / actual_max_velocity_;
        } else {
            // 最大速度に達しない - スケールダウン
            Tv_ = 0.0;
            scaleDownTrajectory();
        }
        
        // Step 5: 各フェーズ時間を設定
        t1_ = Tj_;   // 正ジャーク
        t2_ = Ta_;   // 一定加速
        t3_ = Tj_;   // 負ジャーク
        t4_ = Tv_;   // 一定速度
        t5_ = Tj_;   // 負ジャーク
        t6_ = Ta_;   // 一定減速
        t7_ = Tj_;   // 正ジャーク
        
        total_time_ = t1_ + t2_ + t3_ + t4_ + t5_ + t6_ + t7_;
        
        RCLCPP_INFO(this->get_logger(), 
                    "軌道パラメータ計算完了:\n"
                    "  Tj=%.4f s, Ta=%.4f s, Tv=%.4f s\n"
                    "  総時間=%.4f s\n"
                    "  実際の最大加速度=%.4f m/s², 実際の最大速度=%.4f m/s",
                    Tj_, Ta_, Tv_, total_time_, actual_max_acc_, actual_max_velocity_);
    }
    
    /**
     * @brief 加速フェーズ（Phase 1,2,3）の移動距離を計算
     */
    double calculatePhaseDistance(double tj, double ta, double a_max) {
        // Phase 1: 正ジャーク (0 -> a_max)
        double v1 = 0.5 * max_jerk_ * tj * tj;
        double s1 = (1.0/6.0) * max_jerk_ * tj * tj * tj;
        
        // Phase 2: 一定加速度
        double v2 = v1 + a_max * ta;
        double s2 = v1 * ta + 0.5 * a_max * ta * ta;
        
        // Phase 3: 負ジャーク (a_max -> 0)
        double s3 = v2 * tj + 0.5 * a_max * tj * tj - (1.0/6.0) * max_jerk_ * tj * tj * tj;
        
        return s1 + s2 + s3;
    }
    
    /**
     * @brief 短距離用に軌道をスケールダウン
     */
    void scaleDownTrajectory() {
        // 二分探索で適切な最大速度を見つける
        double v_low = 0.001;
        double v_high = max_velocity_;
        
        for (int iter = 0; iter < 100; iter++) {
            double v_test = (v_low + v_high) / 2.0;
            
            // この速度でのパラメータを計算
            double tj_test, ta_test, a_test;
            double v_at_max_jerk = max_jerk_ * std::pow(max_acceleration_ / max_jerk_, 2);
            
            if (v_test <= v_at_max_jerk) {
                tj_test = std::sqrt(v_test / max_jerk_);
                ta_test = 0.0;
                a_test = max_jerk_ * tj_test;
            } else {
                tj_test = max_acceleration_ / max_jerk_;
                a_test = max_acceleration_;
                ta_test = (v_test - v_at_max_jerk) / max_acceleration_;
            }
            
            // 距離を計算
            double s_accel = calculatePhaseDistanceTest(tj_test, ta_test, a_test);
            double total_dist = 2.0 * s_accel;
            
            if (std::abs(total_dist - target_distance_) < 1e-9) {
                break;
            }
            
            if (total_dist < target_distance_) {
                v_low = v_test;
            } else {
                v_high = v_test;
            }
        }
        
        // 最終パラメータを設定
        actual_max_velocity_ = (v_low + v_high) / 2.0;
        double v_at_max_jerk = max_jerk_ * std::pow(max_acceleration_ / max_jerk_, 2);
        
        if (actual_max_velocity_ <= v_at_max_jerk) {
            Tj_ = std::sqrt(actual_max_velocity_ / max_jerk_);
            Ta_ = 0.0;
            actual_max_acc_ = max_jerk_ * Tj_;
        } else {
            Tj_ = max_acceleration_ / max_jerk_;
            actual_max_acc_ = max_acceleration_;
            Ta_ = (actual_max_velocity_ - v_at_max_jerk) / max_acceleration_;
        }
    }
    
    /**
     * @brief テスト用の距離計算
     */
    double calculatePhaseDistanceTest(double tj, double ta, double a_max) {
        double v1 = 0.5 * max_jerk_ * tj * tj;
        double s1 = (1.0/6.0) * max_jerk_ * tj * tj * tj;
        double v2 = v1 + a_max * ta;
        double s2 = v1 * ta + 0.5 * a_max * ta * ta;
        double s3 = v2 * tj + 0.5 * a_max * tj * tj - (1.0/6.0) * max_jerk_ * tj * tj * tj;
        return s1 + s2 + s3;
    }
    
    /**
     * @brief 指定時刻での目標値を厳密に計算（解析解）
     */
    void computeTrajectoryAtTime(double t, double& pos, double& vel, double& acc) {
        if (t <= 0.0) {
            pos = 0.0;
            vel = 0.0;
            acc = 0.0;
            return;
        }
        
        if (t >= total_time_) {
            pos = target_distance_;
            vel = 0.0;
            acc = 0.0;
            return;
        }
        
        // 各フェーズの境界時刻
        double t_end_1 = t1_;
        double t_end_2 = t1_ + t2_;
        double t_end_3 = t1_ + t2_ + t3_;
        double t_end_4 = t1_ + t2_ + t3_ + t4_;
        double t_end_5 = t1_ + t2_ + t3_ + t4_ + t5_;
        double t_end_6 = t1_ + t2_ + t3_ + t4_ + t5_ + t6_;
        
        // Phase 1: ジャーク増加（加速度上昇）
        if (t <= t_end_1) {
            acc = max_jerk_ * t;
            vel = 0.5 * max_jerk_ * t * t;
            pos = (1.0/6.0) * max_jerk_ * t * t * t;
            return;
        }
        
        // Phase 1の最終状態
        double a1 = max_jerk_ * t1_;
        double v1 = 0.5 * max_jerk_ * t1_ * t1_;
        double s1 = (1.0/6.0) * max_jerk_ * t1_ * t1_ * t1_;
        
        // Phase 2: 一定加速度
        if (t <= t_end_2) {
            double dt = t - t_end_1;
            acc = a1;
            vel = v1 + a1 * dt;
            pos = s1 + v1 * dt + 0.5 * a1 * dt * dt;
            return;
        }
        
        // Phase 2の最終状態
        double a2 = a1;
        double v2 = v1 + a1 * t2_;
        double s2 = s1 + v1 * t2_ + 0.5 * a1 * t2_ * t2_;
        
        // Phase 3: ジャーク減少（加速度減少）
        if (t <= t_end_3) {
            double dt = t - t_end_2;
            acc = a2 - max_jerk_ * dt;
            vel = v2 + a2 * dt - 0.5 * max_jerk_ * dt * dt;
            pos = s2 + v2 * dt + 0.5 * a2 * dt * dt - (1.0/6.0) * max_jerk_ * dt * dt * dt;
            return;
        }
        
        // Phase 3の最終状態
        double v3 = v2 + a2 * t3_ - 0.5 * max_jerk_ * t3_ * t3_;
        double s3 = s2 + v2 * t3_ + 0.5 * a2 * t3_ * t3_ - (1.0/6.0) * max_jerk_ * t3_ * t3_ * t3_;
        
        // Phase 4: 一定速度
        if (t <= t_end_4) {
            double dt = t - t_end_3;
            acc = 0.0;
            vel = v3;
            pos = s3 + v3 * dt;
            return;
        }
        
        // Phase 4の最終状態
        double v4 = v3;
        double s4 = s3 + v3 * t4_;
        
        // Phase 5: ジャーク減少（減速開始）
        if (t <= t_end_5) {
            double dt = t - t_end_4;
            acc = -max_jerk_ * dt;
            vel = v4 - 0.5 * max_jerk_ * dt * dt;
            pos = s4 + v4 * dt - (1.0/6.0) * max_jerk_ * dt * dt * dt;
            return;
        }
        
        // Phase 5の最終状態
        double a5 = -max_jerk_ * t5_;
        double v5 = v4 - 0.5 * max_jerk_ * t5_ * t5_;
        double s5 = s4 + v4 * t5_ - (1.0/6.0) * max_jerk_ * t5_ * t5_ * t5_;
        
        // Phase 6: 一定減速度
        if (t <= t_end_6) {
            double dt = t - t_end_5;
            acc = a5;
            vel = v5 + a5 * dt;
            pos = s5 + v5 * dt + 0.5 * a5 * dt * dt;
            return;
        }
        
        // Phase 6の最終状態
        double a6 = a5;
        double v6 = v5 + a5 * t6_;
        double s6 = s5 + v5 * t6_ + 0.5 * a5 * t6_ * t6_;
        
        // Phase 7: ジャーク増加（減速終了）
        if (t <= total_time_) {
            double dt = t - t_end_6;
            acc = a6 + max_jerk_ * dt;
            vel = v6 + a6 * dt + 0.5 * max_jerk_ * dt * dt;
            pos = s6 + v6 * dt + 0.5 * a6 * dt * dt + (1.0/6.0) * max_jerk_ * dt * dt * dt;
            return;
        }
        
        // 軌道完了
        pos = target_distance_;
        vel = 0.0;
        acc = 0.0;
    }
    
    /**
     * @brief 制御ループ（定期実行）
     */
    void controlLoop() {
        if (!is_running_) {
            return;
        }
        
        if (trajectory_time_ >= total_time_) {
            // 軌道完了 - 最終位置を目標値に固定
            if (!trajectory_completed_) {
                // 最終位置を正確に目標値にセット
                current_position_ = target_distance_;
                current_velocity_ = 0.0;
                current_acceleration_ = 0.0;
                
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.y = 0.0;
                cmd_vel_pub_->publish(cmd_vel);
                
                std_msgs::msg::Bool completed;
                completed.data = true;
                completed_pub_->publish(completed);
                
                // 最終フィードバック
                std_msgs::msg::Float64 pos_msg, vel_msg;
                pos_msg.data = current_position_;
                vel_msg.data = current_velocity_;
                position_pub_->publish(pos_msg);
                velocity_pub_->publish(vel_msg);
                
                RCLCPP_INFO(this->get_logger(), 
                           "軌道完了！ 最終位置: %.6f m (目標: %.6f m), 誤差: %.6f m", 
                           current_position_, target_distance_, 
                           std::abs(current_position_ - target_distance_));
                trajectory_completed_ = true;
                is_running_ = false;
            }
            return;
        }
        
        // 解析的に目標値を計算（累積誤差なし）
        double target_pos, target_vel, target_acc;
        computeTrajectoryAtTime(trajectory_time_, target_pos, target_vel, target_acc);
        
        // 状態更新
        current_position_ = target_pos;
        current_velocity_ = target_vel;
        current_acceleration_ = target_acc;
        
        // cmd_vel メッセージの生成
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = current_velocity_;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
        
        // デバッグ情報の発行
        std_msgs::msg::Float64 pos_msg, vel_msg, acc_msg;
        pos_msg.data = current_position_;
        vel_msg.data = current_velocity_;
        acc_msg.data = current_acceleration_;
        position_pub_->publish(pos_msg);
        velocity_pub_->publish(vel_msg);
        acceleration_pub_->publish(acc_msg);
        
        // 同期された軌道状態を送信 [time, pos, vel, acc]
        std_msgs::msg::Float64MultiArray state_msg;
        state_msg.data = {trajectory_time_, current_position_, current_velocity_, current_acceleration_};
        trajectory_state_pub_->publish(state_msg);
        
        // 時刻を進める
        trajectory_time_ += dt_;
        
        // 定期的にログ出力
        if (static_cast<int>(trajectory_time_ * 10) % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                       "時刻: %.2f s, 位置: %.6f m, 速度: %.6f m/s, 加速度: %.6f m/s^2",
                       trajectory_time_, current_position_, current_velocity_, current_acceleration_);
        }
    }
    
    /**
     * @brief 軌道開始のコールバック
     */
    void startCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && !is_running_) {
            RCLCPP_INFO(this->get_logger(), "軌道開始！");
            is_running_ = true;
            trajectory_time_ = 0.0;
            current_position_ = 0.0;
            current_velocity_ = 0.0;
            current_acceleration_ = 0.0;
            trajectory_completed_ = false;
        } else if (!msg->data && is_running_) {
            RCLCPP_INFO(this->get_logger(), "軌道停止");
            is_running_ = false;
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.y = 0.0;
            cmd_vel_pub_->publish(cmd_vel);
        }
    }
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr acceleration_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr trajectory_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completed_pub_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Parameters
    double target_distance_;
    double max_velocity_;
    double max_acceleration_;
    double max_jerk_;
    double control_frequency_;
    bool auto_start_;
    
    // Trajectory parameters
    double t1_, t2_, t3_, t4_, t5_, t6_, t7_;
    double Tj_, Ta_, Tv_;  // ジャーク時間、加速時間、一定速度時間
    double actual_max_acc_, actual_max_velocity_;
    double total_time_;
    double dt_;
    
    // State
    bool is_running_;
    bool trajectory_completed_;
    double trajectory_time_;
    double current_position_;
    double current_velocity_;
    double current_acceleration_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JerkTrajectoryController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
