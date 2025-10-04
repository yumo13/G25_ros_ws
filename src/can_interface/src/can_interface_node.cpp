#include <arpa/inet.h>
#include <cstdio>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <memory>
#include <string>

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class CANInterfaceNode : public rclcpp::Node {
public:
    CANInterfaceNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("can_interface_node", options) {
        this->declare_parameter<std::string>("interface", "can0");
        this->get_parameter("interface", iface_);

        pub_ = this->create_publisher<can_msgs::msg::Frame>("can_rx", 10);
        sub_ = this->create_subscription<can_msgs::msg::Frame>(
            "can_tx", 10, [this](const can_msgs::msg::Frame &msg) { this->send_frame(msg); });

        if (!open_socket()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CAN socket on %s", iface_.c_str());
            throw std::runtime_error("Failed to open CAN socket");
        }

        timer_ = this->create_wall_timer(1ms, [this]() { this->poll_socket(); });
        RCLCPP_INFO(this->get_logger(), "CAN interface node started on %s", iface_.c_str());
    }

    ~CANInterfaceNode() override {
        if (sock_ >= 0)
            close(sock_);
    }

private:
    bool open_socket() {
        sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_ < 0) {
            perror("socket");
            return false;
        }

        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, iface_.c_str(), IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';
        if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
            perror("ioctl");
            close(sock_);
            sock_ = -1;
            return false;
        }

        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("bind");
            close(sock_);
            sock_ = -1;
            return false;
        }

        // set non-blocking
        int flags = fcntl(sock_, F_GETFL, 0);
        fcntl(sock_, F_SETFL, flags | O_NONBLOCK);

        return true;
    }

    void poll_socket() {
        struct can_frame frame;
        ssize_t n = read(sock_, &frame, sizeof(frame));

        if (n < 0) {
            // no data or error
            return;
        }

        can_msgs::msg::Frame msg;
        // header left default; user can set timestamps if desired
        msg.id = frame.can_id & CAN_EFF_MASK;
        msg.is_extended = (frame.can_id & CAN_EFF_FLAG) != 0;
        msg.is_remote = (frame.can_id & CAN_RTR_FLAG) != 0;
        msg.length = frame.can_dlc;
        for (size_t i = 0; i < 8; ++i)
            msg.data[i] = frame.data[i];

        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Received CAN frame with ID: 0x%X", msg.id);
    }

    void send_frame(const can_msgs::msg::Frame &msg) {
        struct can_frame frame{};
        if (msg.is_extended)
            frame.can_id = msg.id | CAN_EFF_FLAG;
        else
            frame.can_id = msg.id & CAN_SFF_MASK;
        if (msg.is_remote)
            frame.can_id |= CAN_RTR_FLAG;
        frame.can_dlc = msg.length;
        for (size_t i = 0; i < 8; ++i)
            frame.data[i] = msg.data[i];

        ssize_t n = write(sock_, &frame, sizeof(frame));
        if (n < 0) {
            perror("write");
            RCLCPP_ERROR(this->get_logger(), "Failed to write CAN frame");
        }
        RCLCPP_INFO(this->get_logger(), "Sent CAN frame with ID: 0x%X", msg.id);
    }

    std::string iface_{"can0"};
    int sock_{-1};
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CANInterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
