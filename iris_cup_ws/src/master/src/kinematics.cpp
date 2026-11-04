#include <memory>
#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"

class MasterNode : public rclcpp::Node {
public:
  MasterNode() : Node("kinematika_node") {
    // parameter kontrol PID
    kp = declare_parameter<double>("kp", 0.35);
    kd = declare_parameter<double>("kd", 0);
    base_pwm = declare_parameter<int>("base_pwm", 150); // kecepatan dasar mobil
    image_width = declare_parameter<int>("image_width", 640);

    // publisher untuk UDP Sender
    pub_cmd = create_publisher<std_msgs::msg::Int32MultiArray>("/master/geometry_control", 10);

    // publisher untuk basestation
    pub_steering_web = create_publisher<std_msgs::msg::Float32>("/vehicle/steering", 10);
    pub_speed_web = create_publisher<std_msgs::msg::Float32>("/vehicle/speed", 10);

    // subscriber ke data vision
    sub_vision = create_subscription<std_msgs::msg::Int32MultiArray>(
      "/vision/lane_centers", 10,
      std::bind(&MasterNode::vision_callback, this, std::placeholders::_1));

    // subscriber ke feedback dari UDP receiver
    sub_feedback = create_subscription<std_msgs::msg::Int32MultiArray>(
      "/feedback", 10,
      std::bind(&MasterNode::feedback_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Master Node Kinematika Dimulai");
  }

private:
  void feedback_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 4) return;

    int left_ticks = msg->data[0];
    int right_ticks = msg->data[1];
    int vel_x_cms = msg->data[2]; // kecepatan maju/mundur dalam cm/s
    int vel_y_cms = msg->data[3];

    // RCLCPP_INFO(get_logger(), "Kecepatan aktual: %d cm/s | Ticks: L=%d, R=%d", vel_x_cms, left_ticks, right_ticks);

    // simpan ke variabel member 
    current_vel_x = vel_x_cms;

    // publish kecepatan aktual untuk basestation
    std_msgs::msg::Float32 speed_msg;
    speed_msg.data = vel_x_cms;
    pub_speed_web->publish(speed_msg);
  }

  void vision_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    int steer = 0;
    int pwm = 0;

    // jika vision tidak menemukan centroid
    if (msg->data.empty()) {
      steer = last_steer;
      pwm = std::max(0, base_pwm - 30);
    } else {
      // titik tengah frame kamera
      int image_center_x = image_width / 2; 

      // ambil titik x terdekat
      int target_x = msg->data[0]; 

      // hitung error lateral (seberapa jauh target X dari tengah layar)
      int error = target_x - image_center_x;

      // hitung derivative error
      int d_error = error - last_error;

      // hitung steering dengan PD
      steer = static_cast<int>((kp * error) + (kd * d_error));
      steer = std::clamp(steer, -90, 90);

      // hitung pwm
      pwm = base_pwm - static_cast<int>(std::abs(steer) * 1.5);

      // tambah tenaga kalo tersangkut
      if (current_vel_x < 10 && pwm > 0) {
        pwm += 30; 
      }
      pwm = std::clamp(pwm, 0, 255); 

      // simpan memori
      last_steer = steer;
      last_error = error;
    }

      // publish ke UDP sender [pwm, steer]
      std_msgs::msg::Int32MultiArray cmd_msg;
      cmd_msg.data = {pwm, steer};
      pub_cmd->publish(cmd_msg);

      // publish steering angle ke basestation
      std_msgs::msg::Float32 steering_web_msg;
      steering_web_msg.data = static_cast<float>(steer);
      pub_steering_web->publish(steering_web_msg);
  }

  double kp;
  double kd;
  int base_pwm;
  int image_width;

  int last_error = 0;
  int last_steer = 0;
  int current_vel_x = 0;

  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_cmd;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_steering_web;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_speed_web;

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_vision;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_feedback;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MasterNode>());
  rclcpp::shutdown();
  return 0;
}