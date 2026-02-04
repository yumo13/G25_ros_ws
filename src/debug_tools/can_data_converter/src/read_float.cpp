#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <can_msgs/msg/frame.hpp>

#include <cstring>
#include <vector>
#include <cstdint>

class ReadFloatNode : public rclcpp::Node {
public:
  ReadFloatNode() : Node("read_float_node") {
    this->declare_parameter<int64_t>("can_id", static_cast<int64_t>(0x300));
    int64_t tmp_id = 0;
    this->get_parameter("can_id", tmp_id);
    can_id_ = static_cast<uint32_t>(tmp_id);

    pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("float_out", 10);
    sub_ = this->create_subscription<can_msgs::msg::Frame>(
        "can_rx", 10,
        [this](const can_msgs::msg::Frame &msg) {
          this->frame_callback(msg);
        });

    RCLCPP_INFO(this->get_logger(), "read_float_node started, publishing to float_out with id 0x%X", can_id_);
  }

private:
  void frame_callback(const can_msgs::msg::Frame &msg) {
    if (msg.id != can_id_) {
      RCLCPP_WARN(this->get_logger(), "Received message with unexpected CAN ID 0x%X", msg.id);
      return;
    }

    float value_1 = 0.0f, value_2 = 0.0f;
    std::memcpy(&value_1, msg.data.data(), sizeof(float));
    std::memcpy(&value_2, msg.data.data() + sizeof(float), sizeof(float));
    RCLCPP_INFO(this->get_logger(), "Received float %f from CAN ID 0x%X", value_1, can_id_);

    std_msgs::msg::Float32MultiArray float_msg;
    float_msg.data.push_back(value_1);
    float_msg.data.push_back(value_2);
    pub_->publish(float_msg);
  }

  uint32_t can_id_ = 0x123;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReadFloatNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
