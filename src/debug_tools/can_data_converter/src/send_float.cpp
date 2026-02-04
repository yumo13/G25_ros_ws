#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <cstdint>
#include <cstring>
#include <vector>

/**
 * @brief Float送信ノード
 * ROSトピックからfloatを受け取り、CANフレームとして送信
 */
class SendFloatNode : public rclcpp::Node {
public:
    SendFloatNode() : Node("send_float_node") {
        this->declare_parameter<int64_t>("can_id", static_cast<int64_t>(0x201));
        int64_t tmp_id = 0;
        this->get_parameter("can_id", tmp_id);
        can_id_ = static_cast<uint32_t>(tmp_id);

        pub_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", 10);
        sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "float_in", 10, [this](const std_msgs::msg::Float32 &msg) { this->float_callback(msg); });

        RCLCPP_INFO(this->get_logger(), "send_float_node started, publishing to can_tx with id 0x%X", can_id_);
    }

private:
    void float_callback(const std_msgs::msg::Float32 &msg) {
        float value = msg.data;

        std::array<uint8_t, 4> bytes;
        static_assert(sizeof(float) == 4, "Unexpected float size");
        std::memcpy(bytes.data(), &value, sizeof(float));

        can_msgs::msg::Frame frame_msg;
        frame_msg.id = can_id_;
        frame_msg.is_extended = false;
        frame_msg.is_remote = false;
        frame_msg.length = 4;
        for (size_t i = 0; i < 4; ++i) {
            frame_msg.data[i] = bytes[i];
        }
        for (size_t i = 4; i < 8; ++i)
            frame_msg.data[i] = 0;

        pub_->publish(frame_msg);
        RCLCPP_INFO(this->get_logger(), "Published float %f as CAN id 0x%X", value, can_id_);
    }

    uint32_t can_id_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SendFloatNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
