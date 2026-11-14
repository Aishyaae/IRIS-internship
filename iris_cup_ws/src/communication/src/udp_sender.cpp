#include <arpa/inet.h>
#include <endian.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

class UdpSenderNode : public rclcpp::Node {
public:
  UdpSenderNode() : Node("udp_sender_node") {
    // parameter
    host = declare_parameter<std::string>("host", "192.168.54.252"); // IP ESP32
    port = declare_parameter<int>("port", 4001);
    rate_hz = declare_parameter<int>("rate_hz", 20);
    timeout_ms = declare_parameter<int>("cmd_timeout_ms", 1000);

    setup_socket();

    // subscribe ke master
    sub = create_subscription<std_msgs::msg::Int32MultiArray>(
        "/master/geometry_control", 10,
        std::bind(&UdpSenderNode::master_callback, this, std::placeholders::_1));

    // timer pengiriman periodik
    timer = create_wall_timer(
        std::chrono::milliseconds(1000 / std::max(1, rate_hz)),
        std::bind(&UdpSenderNode::timer_callback, this));

    last_cmd_time = now();
    RCLCPP_INFO(get_logger(), "Mengirim UDP Biner (FRANA) ke ESP32 di %s:%d pada %d Hz", host.c_str(), port, rate_hz);
  }

  // destructor (ctrl+c)
  ~UdpSenderNode() override {
    if (sock >= 0) {
      pwm = 0;
      send_packet();
      RCLCPP_INFO(get_logger(), "Stopped");
      close(sock);
    }
  }

private:
  static constexpr char kHeader[5] = {'F', 'R', 'A', 'N', 'A'};
  static constexpr size_t kPacketSize = 13; // 5 (header) + 4 (pwm) + 4 (steer)

  void setup_socket() {
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
      throw std::runtime_error("Failed to create socket");
    }

    addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
      close(sock);
      sock = -1;
      throw std::runtime_error("Invalid host: " + host);
    }
  }

  // isi buffer biner 13 byte: "FRANA" | pwm (int32 LE) | steer (int32 LE)
  void build_packet(uint8_t (&buf)[kPacketSize], int32_t pwm_val, int32_t steer_val) {
    pwm_val = std::clamp<int32_t>(pwm_val, -9999, 9999);
    steer_val = std::clamp<int32_t>(steer_val, 50, 120);

    uint32_t pwm_le = htole32(static_cast<uint32_t>(pwm_val));
    uint32_t steer_le = htole32(static_cast<uint32_t>(steer_val));

    // menyusun paket byte per byte
    std::memcpy(buf, kHeader, sizeof(kHeader));
    std::memcpy(buf + 5, &pwm_le, sizeof(pwm_le));
    std::memcpy(buf + 9, &steer_le, sizeof(steer_le));
  }

  void send_packet() {
    if (sock < 0) return;
    uint8_t packet[kPacketSize];
    build_packet(packet, pwm, steer);

    if (sendto(sock, packet, sizeof(packet), 0, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "sendto failed");
    }
  }

  void master_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 2) {
      RCLCPP_WARN(get_logger(), "cmd_motor membutuhkan 2 nilai: [pwm, steer]");
      return;
    }
    pwm = msg->data[0];
    steer = msg->data[1];
    last_cmd_time = now();
  }

  void timer_callback() {
    // safety timeout: jika tidak ada perintah baru maka berhenti
    if ((now() - last_cmd_time).nanoseconds() > timeout_ms * 1000000LL) {
      pwm = 0;
    }
    send_packet();
  }

  int sock = -1;
  sockaddr_in addr{};
  std::string host;
  int port;
  int rate_hz;
  int timeout_ms;

  int32_t pwm = 0;
  int32_t steer = 0;
  rclcpp::Time last_cmd_time{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub;
  rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UdpSenderNode>());
  rclcpp::shutdown();
  return 0;
}