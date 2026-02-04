#include <chrono>
#include <memory>
#include <string>

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

using namespace std::chrono_literals;

/**
 * @brief CANデータ変換ノード
 * 指定されたCAN IDのフレームを受信し、データをint64に変換して配信
 */
class CANDataConverterNode : public rclcpp::Node {
public:
    CANDataConverterNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("can_data_converter_node", options) {

        // パラメータ宣言
        this->declare_parameter<int>("target_can_id", 273);
        this->declare_parameter<std::string>("output_topic", "can_data_int64");
        this->declare_parameter<bool>("little_endian", false);

        // パラメータ取得
        this->get_parameter("target_can_id", target_can_id_);
        this->get_parameter("output_topic", output_topic_);
        this->get_parameter("little_endian", little_endian_);

        pub_ = this->create_publisher<std_msgs::msg::Int64>(output_topic_, 10);
        sub_ = this->create_subscription<can_msgs::msg::Frame>(
            "can_rx", 10, [this](const can_msgs::msg::Frame &msg) { this->process_can_frame(msg); });

        RCLCPP_INFO(this->get_logger(), "CAN Data Converter started - filtering ID: 0x%X, publishing to: %s",
                    target_can_id_, output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Endianness: %s", little_endian_ ? "Little Endian" : "Big Endian");
    }

private:
    void process_can_frame(const can_msgs::msg::Frame &msg) {
        if (msg.id != static_cast<uint32_t>(target_can_id_)) {
            return;
        }

        if (msg.length == 0 || msg.length > 8) {
            RCLCPP_WARN(this->get_logger(), "Invalid data length: %d for CAN ID 0x%X", msg.length, msg.id);
            return;
        }

        int64_t converted_value = convert_to_int64(msg.data, msg.length);

        auto int64_msg = std_msgs::msg::Int64();
        int64_msg.data = converted_value;
        pub_->publish(int64_msg);

        RCLCPP_DEBUG(this->get_logger(), "CAN ID 0x%X: converted %d bytes to int64: %ld", msg.id, msg.length,
                     converted_value);
    }

    int64_t convert_to_int64(const std::array<uint8_t, 8> &data, uint8_t length) {
        int64_t result = 0;

        if (little_endian_) {
            // リトルエンディアン: 下位バイトから上位バイト
            for (int i = length - 1; i >= 0; --i) {
                result = (result << 8) | data[i];
            }
        } else {
            // ビッグエンディアン: 上位バイトから下位バイト
            for (int i = 0; i < length; ++i) {
                result = (result << 8) | data[i];
            }
        }

        // 負の数の符号拡張
        if (length < 8 && (data[little_endian_ ? length - 1 : 0] & 0x80)) {
            result |= (0xFFFFFFFFFFFFFFFFLL << (length * 8));
        }

        return result;
    }

    int target_can_id_;
    std::string output_topic_;
    bool little_endian_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CANDataConverterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}