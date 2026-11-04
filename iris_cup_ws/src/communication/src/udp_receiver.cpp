#include <arpa/inet.h>
#include <endian.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

class UdpReceiverNode : public rclcpp::Node {
public:
  UdpReceiverNode() : Node("udp_receiver_node") {
    // parameter (bisa diubah dari launch file / command line)
    port = declare_parameter<int>("port", 12152);
    poll_rate_hz = declare_parameter<int>("poll_rate_hz", 100);
    topic = declare_parameter<std::string>("topic", "/feedback");

    setup_socket();

    // publisher: data = [left_ticks, right_ticks, vel_x_cms, vel_y_cms]
    pub = create_publisher<std_msgs::msg::Int32MultiArray>(topic, 10);

    // timer polling (pengganti loop recvfrom yang memblokir)
    timer = create_wall_timer(
        std::chrono::milliseconds(1000 / std::max(1, poll_rate_hz)),
        std::bind(&UdpReceiverNode::timer_callback, this));

    RCLCPP_INFO(get_logger(), "Listening for feedback on UDP port %d -> topic %s", port, topic.c_str());
  }

  // dipanggil saat node dihancurkan (Ctrl+C / shutdown)
  ~UdpReceiverNode() override {
    if (sock >= 0) {
      close(sock);
      RCLCPP_INFO(get_logger(), "Stopped");
    }
  }

private:
  static constexpr char kHeader[4] = {'I', 'R', 'I', 'S'};
  static constexpr size_t kPacketSize = 20;

  struct Feedback {
    int32_t left_ticks;
    int32_t right_ticks;
    int32_t vel_x_cms;
    int32_t vel_y_cms;
  };

  // membuat socket UDP, bind ke port, dan set mode non-blocking
  void setup_socket() {
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
      throw std::runtime_error("Failed to create socket");
    }

    // izinkan bind ulang cepat setelah node di-restart
    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      close(sock);
      sock = -1;
      throw std::runtime_error("bind failed on port " + std::to_string(port) + " (port sudah dipakai proses lain?)");
    }
  }

  // baca int32 little-endian dari buffer byte
  static int32_t read_i32_le(const uint8_t *src) {
    uint32_t raw;
    std::memcpy(&raw, src, sizeof(raw));
    return static_cast<int32_t>(le32toh(raw));
  }

  // validasi header + panjang, lalu isi struct Feedback
  static bool parse_packet(const uint8_t *buf, size_t len, Feedback &out) {
    if (len < kPacketSize || std::memcmp(buf, kHeader, sizeof(kHeader)) != 0) {
      return false;
    }
    out.left_ticks = read_i32_le(buf + 4);
    out.right_ticks = read_i32_le(buf + 8);
    out.vel_x_cms = read_i32_le(buf + 12);
    out.vel_y_cms = read_i32_le(buf + 16);
    return true;
  }

  // kosongkan semua paket yang menunggu di socket, publish tiap paket valid
  void timer_callback() {
    uint8_t buf[64];
    while (true) {
      ssize_t n = recvfrom(sock, buf, sizeof(buf), MSG_DONTWAIT, nullptr, nullptr);
      if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
          RCLCPP_ERROR(get_logger(), "recvfrom failed: %s", std::strerror(errno));
        }
        break;  // tidak ada paket lagi
      }

      Feedback fb;
      if (!parse_packet(buf, static_cast<size_t>(n), fb)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring malformed packet (%zd bytes)", n);
        continue;
      }
      publish_feedback(fb);
    }
  }

  void publish_feedback(const Feedback &fb) {
    std_msgs::msg::Int32MultiArray msg;
    msg.data = {fb.left_ticks, fb.right_ticks, fb.vel_x_cms, fb.vel_y_cms};
    pub->publish(msg);
  }

  // member
  int sock = -1;
  int port;
  int poll_rate_hz;
  std::string topic;

  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub;
  rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UdpReceiverNode>());
  rclcpp::shutdown();
  return 0;
}