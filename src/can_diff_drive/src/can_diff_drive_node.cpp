#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <can_msgs/msg/frame.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <cstring>
#include <cstdint>

class CanDiffDrive : public rclcpp::Node {
public:
  CanDiffDrive() : Node("can_diff_drive") {
    // parameters
    this->declare_parameter<int64_t>("left_can_id", static_cast<int64_t>(0x201));
    this->declare_parameter<int64_t>("right_can_id", static_cast<int64_t>(0x211));
    this->declare_parameter<double>("wheel_base", 0.5); // meters
    this->declare_parameter<double>("max_motor", 1.0); // 0.0 - 1.0 scale for motor command
    this->declare_parameter<double>("cmd_timeout", 1.0); // seconds - timeout for cmd_vel messages

    int64_t l = 0, r = 0;
    this->get_parameter("left_can_id", l);
    this->get_parameter("right_can_id", r);
    left_can_id_ = static_cast<uint32_t>(l);
    right_can_id_ = static_cast<uint32_t>(r);
    this->get_parameter("wheel_base", wheel_base_);
    this->get_parameter("max_motor", max_motor_);
    this->get_parameter("cmd_timeout", cmd_timeout_);

    pub_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", 10);
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [this](const geometry_msgs::msg::Twist &msg) { cmd_vel_cb(msg); });

    // Debug: publish motor commands
    motor_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("motor_commands", 10);

    // Create timeout timer - check every 100ms
    timeout_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), [this]() { timeout_check(); });

    // Initialize last cmd time to current time
    last_cmd_time_ = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "can_diff_drive started; left_id=0x%X right_id=0x%X max_motor=%f timeout=%fs",
                left_can_id_, right_can_id_, max_motor_, cmd_timeout_);
  }

private:
  void cmd_vel_cb(const geometry_msgs::msg::Twist &msg) {
    // Update last command time
    last_cmd_time_ = this->get_clock()->now();

    double linear = msg.linear.y; // forward m/s
    double angular = msg.angular.z; // yaw rad/s

    // Simple differential kinematics: v_left = v - (omega * wheel_base/2), v_right = v + (omega * wheel_base/2)
    double v_left = linear - (angular * wheel_base_ / 2.0);
    double v_right = linear + (angular * wheel_base_ / 2.0);

    // Convert to normalized motor commands in range [-1, 1] using max_motor_ as scaling factor
    // We assume user provides velocities in range that map correctly; for safety, clamp by max_motor_
    double left_cmd = clamp(v_left, -max_motor_, max_motor_);
    double right_cmd = clamp(-v_right, -max_motor_, max_motor_);

    // Pack each as 4-byte float into CAN frame payload
    publish_motor(left_can_id_, static_cast<float>(left_cmd));
    publish_motor(right_can_id_, static_cast<float>(right_cmd));

    RCLCPP_INFO(this->get_logger(), "cmd_vel received: linear=%.4f angular=%.4f => left_cmd=%.4f right_cmd=%.4f",
                 linear, angular, left_cmd, right_cmd);

    // Publish motor commands for debugging (rqt_plot)
    publish_motor_commands(static_cast<float>(left_cmd), static_cast<float>(right_cmd));
  }

  void publish_motor(uint32_t can_id, float value) {
    std::array<uint8_t, 4> bytes;
    static_assert(sizeof(float) == 4, "float size must be 4 bytes");
    std::memcpy(bytes.data(), &value, sizeof(float));

    can_msgs::msg::Frame frame;
    frame.id = can_id;
    frame.is_extended = false;
    frame.is_remote = false;
    frame.length = 4;
    for (size_t i = 0; i < 4; ++i) frame.data[i] = bytes[i];
    for (size_t i = 4; i < 8; ++i) frame.data[i] = 0;

    pub_->publish(frame);
    RCLCPP_DEBUG(this->get_logger(), "Published motor cmd %f to CAN ID 0x%X", value, can_id);
  }

  double clamp(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  }

  void publish_motor_commands(float left_cmd, float right_cmd) {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data.resize(2);
    msg.data[0] = left_cmd;
    msg.data[1] = right_cmd;
    motor_cmd_pub_->publish(msg);
    RCLCPP_DEBUG(this->get_logger(), "Motor commands: left=%.4f, right=%.4f", left_cmd, right_cmd);
  }

  void timeout_check() {
    auto now = this->get_clock()->now();
    auto time_since_last_cmd = (now - last_cmd_time_).seconds();
    
    if (time_since_last_cmd > cmd_timeout_) {
      // Send zero commands for safety
      publish_motor(left_can_id_, 0.0f);
      publish_motor(right_can_id_, 0.0f);
      publish_motor_commands(0.0f, 0.0f);
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "cmd_vel timeout (%.2fs) - sending zero commands", time_since_last_cmd);
    }
  }

  uint32_t left_can_id_{0x201};
  uint32_t right_can_id_{0x211};
  double wheel_base_{0.5};
  double max_motor_{1.0};
  double cmd_timeout_{1.0};

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  rclcpp::Time last_cmd_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CanDiffDrive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
