#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <can_msgs/msg/frame.hpp>

#include <fstream>
#include <iomanip>
#include <cstring>
#include <chrono>
#include <sstream>
#include <filesystem>

class MotorDataLogger : public rclcpp::Node {
public:
  MotorDataLogger() : Node("motor_data_logger") {
    // パラメータ宣言
    this->declare_parameter<std::string>("output_file", "motor_data.csv");
    this->declare_parameter<int64_t>("speed_can_id", static_cast<int64_t>(0x300));
    this->declare_parameter<int64_t>("voltage_can_id", static_cast<int64_t>(0x301));
    this->declare_parameter<bool>("auto_start", false);
    
    // パラメータ取得
    this->get_parameter("output_file", output_file_);
    int64_t speed_id_param = 0;
    int64_t voltage_id_param = 0;
    this->get_parameter("speed_can_id", speed_id_param);
    this->get_parameter("voltage_can_id", voltage_id_param);
    this->get_parameter("auto_start", auto_start_);
    
    speed_can_id_ = static_cast<uint32_t>(speed_id_param);
    voltage_can_id_ = static_cast<uint32_t>(voltage_id_param);
    
    motor_cmd_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "motor_commands", 10,
        std::bind(&MotorDataLogger::motorCommandCallback, this, std::placeholders::_1));
    
    can_rx_sub_ = this->create_subscription<can_msgs::msg::Frame>(
        "can_rx", 100,
        std::bind(&MotorDataLogger::canRxCallback, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&MotorDataLogger::timerCallback, this));
    
    is_logging_ = auto_start_;
    start_time_ = this->get_clock()->now();
    
    RCLCPP_INFO(this->get_logger(),
                "MotorDataLogger started\n"
                "  Output file: %s\n"
                "  Speed CAN ID: 0x%03X\n"
                "  Voltage CAN ID: 0x%03X\n"
                "  Auto start: %s",
                output_file_.c_str(), speed_can_id_, voltage_can_id_,
                auto_start_ ? "true" : "false");
    
    if (is_logging_) {
      openCSVFile();
    }
  }
  
  ~MotorDataLogger() {
    closeCSVFile();
  }
  
private:
  void openCSVFile() {
    // logs フォルダが存在しない場合は作成
    std::filesystem::path log_dir = "logs";
    if (!std::filesystem::exists(log_dir)) {
      std::filesystem::create_directories(log_dir);
      RCLCPP_INFO(this->get_logger(), "Created logs directory");
    }
    
    // タイムスタンプ付きファイル名を生成
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "logs/motor_data_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".csv";
    
    csv_file_.open(ss.str(), std::ios::out);
    if (!csv_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", ss.str().c_str());
      return;
    }
    
    // ヘッダー書き込み
    csv_file_ << "timestamp,target_left,target_right,actual_left,actual_right,voltage_left,voltage_right\n";
    csv_file_.flush();
    
    RCLCPP_INFO(this->get_logger(), "CSV file opened: %s", ss.str().c_str());
  }
  
  void closeCSVFile() {
    if (csv_file_.is_open()) {
      csv_file_.close();
      RCLCPP_INFO(this->get_logger(), "CSV file closed");
    }
  }
  
  void motorCommandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (!is_logging_) {
      is_logging_ = true;
      start_time_ = this->get_clock()->now();
      openCSVFile();
    }
    
    if (msg->data.size() >= 2) {
      target_left_ = msg->data[0];
      target_right_ = msg->data[1];
    }
    
    last_cmd_time_ = this->get_clock()->now();
  }
  
  void canRxCallback(const can_msgs::msg::Frame::SharedPtr msg) {
    if (!is_logging_) {
      return;
    }
    
    if (msg->id == speed_can_id_ && msg->length >= 8) {
      // 速度データ：左右2つのfloat (4バイトずつ)
      std::memcpy(&actual_left_, &msg->data[0], sizeof(float));
      std::memcpy(&actual_right_, &msg->data[4], sizeof(float));
      has_speed_data_ = true;
    }
    else if (msg->id == voltage_can_id_ && msg->length >= 8) {
      // 電圧データ：左右2つのfloat (4バイトずつ)
      std::memcpy(&voltage_left_, &msg->data[0], sizeof(float));
      std::memcpy(&voltage_right_, &msg->data[4], sizeof(float));
      has_voltage_data_ = true;
    }
  }
  
  void timerCallback() {
    if (!is_logging_ || !csv_file_.is_open()) {
      return;
    }
    
    // データが揃っていればCSVに書き込み
    if (has_speed_data_ || has_voltage_data_) {
      double elapsed = (this->get_clock()->now() - start_time_).seconds();
      
      csv_file_ << std::fixed << std::setprecision(6)
                << elapsed << ","
                << target_left_ << ","
                << target_right_ << ","
                << actual_left_ << ","
                << actual_right_ << ","
                << voltage_left_ << ","
                << voltage_right_ << "\n";
      
      // 定期的にフラッシュ (100Hzなので100回=1秒ごと)
      static int flush_counter = 0;
      if (++flush_counter >= 100) {
        csv_file_.flush();
        flush_counter = 0;
      }
    }
  }
  
  std::string output_file_;
  uint32_t speed_can_id_;
  uint32_t voltage_can_id_;
  bool auto_start_;
  
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motor_cmd_sub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_rx_sub_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  
  bool is_logging_{false};
  rclcpp::Time start_time_;
  rclcpp::Time last_cmd_time_;
  std::ofstream csv_file_;
  
  double target_left_{0.0};
  double target_right_{0.0};
  float actual_left_{0.0f};
  float actual_right_{0.0f};
  float voltage_left_{0.0f};
  float voltage_right_{0.0f};
  bool has_speed_data_{false};
  bool has_voltage_data_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotorDataLogger>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
