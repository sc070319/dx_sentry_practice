#include "uart_transporter.hpp"
#include "uart_transporter.cpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cstring>
#include <vector>
#include <cmath>
#include <cstdint>

// ================= CRC16 (电控实测: poly=0x8408(反射), init=0xFFFF, 小端存放) =================
// 注: 不是 RM 标准 CRC(0xA001); 实测覆盖 [0:27], 1642/1642 帧通过
static uint16_t crc16_uplink(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]);
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x0001) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408)
                           : static_cast<uint16_t>(crc >> 1);
    }
  }
  return crc;
}

// ================= 上行帧布局 (电控实测协议 2026-09-16, 29 字节 @ ~820Hz) =================
//   [0-1]   'S' 'P'      帧头
//   [2]     0x01         常量(待电控确认)
//   [3-6]   yaw (float)  弧度, 小端          ← 大 yaw
//   [7-10]  float        恒 0(待确认)
//   [11-14] float        ±0.0003(待确认, 疑似 pitch/roll 或角速度)
//   [15-22] 全 0
//   [23]    0x02         常量
//   [24-26] 0
//   [27-28] CRC16        小端, 覆盖 [0:27]
// 若电控固件再变更, 只需改以下 4 处: 帧长/偏移/覆盖范围/CRC 多项式
static constexpr uint8_t HEAD0       = 'S';
static constexpr uint8_t HEAD1       = 'P';
static constexpr size_t  UPLINK_LEN  = 29;   // 整帧字节数
static constexpr size_t  YAW_OFFSET  = 3;    // yaw 起始字节
static constexpr size_t  CRC_COVER   = 27;   // 参与 CRC 的字节数(= UPLINK_LEN - 2)

class CmdVelSubscriber : public rclcpp::Node
{
public:
  CmdVelSubscriber()
  : Node("cmd_vel_subscriber"),
    port_(this->declare_parameter<std::string>("port", "/dev/ttyACM0")),  // 串口设备, 可 -p port:=
    debug_tx_(this->declare_parameter<bool>("debug_tx", false)),         // true=打印发给电控的帧
    uart_(port_, 115200)
  {
    // ---------- 下行: 订阅速度 ----------
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "aft_cmd_vel", 10,
      std::bind(&CmdVelSubscriber::topic_callback, this, std::placeholders::_1));

    // ---------- 上行: 发布大 yaw ----------
    yaw_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/yaw", 10);

    // ---------- 首次开串口(只开这一次) ----------
    if (uart_.open()) {
      RCLCPP_INFO(this->get_logger(), "UART opened: %s @115200", port_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "UART open failed (will retry on demand): %s",
                  uart_.errorMessage().c_str());
    }

    // ---------- 定时读串口 ----------
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5), std::bind(&CmdVelSubscriber::read_loop, this));
  }

  ~CmdVelSubscriber() { uart_.close(); }

private:
  // ================= 惰性重连: 没开就尝试开 =================
  bool ensureOpen()
  {
    if (uart_.isOpen()) return true;
    if (uart_.open()) {
      RCLCPP_INFO(this->get_logger(), "UART reconnected");
      return true;
    }
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "UART unavailable: %s", uart_.errorMessage().c_str());
    return false;
  }

  // ================= 下行: 发速度给电控 (0xA5 帧, 15 字节) =================
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    uint8_t header = 0xA5;
    float vx = msg->linear.x;
    float vy = msg->linear.y;
    float vz = msg->linear.z;        // 自旋速度(由 fake_vel_transform 写入)

    uint8_t packet[15];
    packet[0] = header;
    std::memcpy(&packet[1], &vx, sizeof(float));
    std::memcpy(&packet[5], &vy, sizeof(float));
    std::memcpy(&packet[9], &vz, sizeof(float));

    uint16_t checksum = header;                       // 累加校验
    for (int i = 1; i < 13; ++i) checksum += packet[i];
    std::memcpy(&packet[13], &checksum, sizeof(uint16_t));

    if (!ensureOpen()) return;
    int n = uart_.writeBuffer(packet, sizeof(packet));
    if (n != static_cast<int>(sizeof(packet))) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "UART 发送不完整: %d/%zu 字节", n, sizeof(packet));
    } else if (debug_tx_) {
      char hexbuf[64] = {0};
      char *p = hexbuf;
      for (size_t i = 0; i < sizeof(packet) && (p - hexbuf) < 60; ++i) {
        p += std::snprintf(p, 4, "%02X ", packet[i]);
      }
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                           "TX 0xA5: vx=%+.3f vy=%+.3f vz=%+.3f [%s]", vx, vy, vz, hexbuf);
    }
  }

  // ================= 上行: 收电控回传的 yaw (SP 帧, 29 字节) =================
  void read_loop()
  {
    if (!ensureOpen()) return;

    uint8_t buf[64];
    int n = uart_.read(buf, sizeof(buf));
    if (n < 0) { uart_.close(); return; }     // 读错误 → 断开, 下次重连
    if (n == 0) return;
    rx_cache_.insert(rx_cache_.end(), buf, buf + n);

    while (rx_cache_.size() >= UPLINK_LEN) {
      // ① 两字节帧头 "SP"
      if (rx_cache_[0] != HEAD0 || rx_cache_[1] != HEAD1) {
        rx_cache_.erase(rx_cache_.begin());   // 不是帧头 → 丢1字节滑动继续找
        continue;
      }

      // ② CRC16 校验: 前 CRC_COVER 字节算出的 CRC, 与末尾 2 字节(小端)比较
      uint16_t crc_calc = crc16_uplink(rx_cache_.data(), CRC_COVER);
      uint16_t crc_recv = static_cast<uint16_t>(
        rx_cache_[UPLINK_LEN - 2] | (rx_cache_[UPLINK_LEN - 1] << 8));
      if (crc_calc != crc_recv) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "CRC mismatch");
        rx_cache_.erase(rx_cache_.begin());   // 校验错 → 丢1字节重找
        continue;
      }

      // ③ 取 yaw (float, 从 YAW_OFFSET 起 4 字节)
      float yaw_val = 0.0f;
      std::memcpy(&yaw_val, &rx_cache_[YAW_OFFSET], sizeof(float));

      // ④ 单位:弧度
      double yaw_rad = static_cast<double>(yaw_val);

      std_msgs::msg::Float64 m;
      m.data = yaw_rad;
      yaw_publisher_->publish(m);

      // ⑤ 消费掉这一帧
      rx_cache_.erase(rx_cache_.begin(), rx_cache_.begin() + UPLINK_LEN);
    }
  }

  std::string port_;
  bool debug_tx_;
  UartTransporter uart_;
  std::vector<uint8_t> rx_cache_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelSubscriber>());
  rclcpp::shutdown();
  return 0;
}