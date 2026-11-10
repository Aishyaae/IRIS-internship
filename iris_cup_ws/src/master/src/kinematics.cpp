
#include <memory>
#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std;

class MasterNode : public rclcpp::Node {
public:
  MasterNode() : Node("kinematika_node") {
    // parameter robot
    wheel_base = declare_parameter<double>("wheel_base", 0.130);

    // parameter kamera
    image_width = declare_parameter<int>("image_width", 640);
    image_height = declare_parameter<int>("image_height", 480);

    // parameter kecepatan dan PID
    target_vel = declare_parameter<int>("target_vel", 50); // target kecepatan 50cm/s
    current_pwm = declare_parameter<int>("start_pwm", 1500);

    kp = declare_parameter<double>("kp", 0.35);
    ki = declare_parameter<double>("ki", 0.1);
    kd = declare_parameter<double>("kd", 0.5);

    // publisher untuk UDP Sender
    pub_cmd = create_publisher<std_msgs::msg::Int32MultiArray>("/master/geometry_control", 10);

    // publisher untuk basestation
    pub_steering_web = create_publisher<std_msgs::msg::Float32>("/vehicle/steering", 10);
    pub_speed_web = create_publisher<std_msgs::msg::Float32>("/vehicle/speed", 10);

    // subscriber ke data vision
    sub_vision = create_subscription<std_msgs::msg::Int32MultiArray>(
      "/vision/lane_centers", 10,
      bind(&MasterNode::vision_callback, this, placeholders::_1));

    // subscriber ke feedback dari UDP receiver
    sub_feedback = create_subscription<std_msgs::msg::Int32MultiArray>(
      "/feedback", 10,
      bind(&MasterNode::feedback_callback, this, placeholders::_1));

    RCLCPP_INFO(get_logger(), "Master Node Kinematika Dimulai");
  }

private:
  void pixel_convertion(int target_x, int target_y, double &dx, double &dy) {
    int center_x = image_width / 2;
    int bottom_y = image_height;

    // double xm_per_pix = 1.0 / 400/0; // 1 meter = 400 pixel
    // double ym_per_pix = 1.0 / 400/0;
    double dummy_scale = 0.001;

    dx = (target_x - center_x) * dummy_scale;
    dy = (bottom_y - target_y) * dummy_scale;
  }

  double pure_pursuit(double dx, double dy) {
    double Ld = hypot(dx, dy);

    if (Ld < 0.001) return 0.0;

    double theta = atan2(dx, dy);
    double k = (2.0 * sin(theta) / Ld);

    double virtual_steering = atan(wheel_base * k);
    return virtual_steering * (180.0 / M_PI); // ubah ke derajat
  }

  void vel_pid(int actual_vel) {
    int error = target_vel - actual_vel;
    
    integral_vel_error += error;
    integral_vel_error = clamp(integral_vel_error, -500, 500);

    int d_error = error - last_vel_error;

    double p = kp * error;
    double i = ki * integral_vel_error;
    double d = kd * d_error;

    int correction = static_cast<int>(p + i + d);

    current_pwm += correction;
    current_pwm = clamp(current_pwm, -9999, 9999);

    last_vel_error = error;
    return current_pwm;
  }

  void feedback_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 4) return;

    current_vel_x = msg->data[2]; // kecepatan maju

    std_msgs::msg::Float32 speed_msg;
    speed_msg.data = static_cast<float>(current_vel_x);
    pub_speed_web->publish(speed_msg);
  }

  void vision_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    int pwm = 0;
    int steer = 0;

    // jika vision tidak mendeteksi garis
    if (msg->data.empty()) {
      steer= last_steer; // pertahankan sudut terakhir
      pwm = max(0, current_pwm - 200); // kurangi kecepatan perlahan
    } else {
      int target_x = msg->data[0];
      int target_y = msg->data[1];

      // konversi pixel
      double dx = 0.0;
      double dy = 0.0;
      pixel_convertion(target_x, target_y, dx, dy);

      // pure pursuit
      steer = static_cast<int>pure_pursuit(dx, dy);
      steer = clamp(steer, -35, 35);

      // PID kecepatan
      pwm = vel_pid(current_vel_x);

      // slow down saat belok
      if (abs(steer) > 20) {
        pwm = max(0, pwm - 300);
      }

      last_steer = steer;

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

  double wheel_base;
  int image_width, image_height, servo_offset;

  int target_vel, current_vel_x, current_pwm;
  double kp, ki, kd;
  int integral_vel_error = 0;
  int last_vel_error = 0;
  int last_steer = 0;

  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_cmd;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_steering_web;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_speed_web;

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_vision;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_feedback;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<MasterNode>());
  rclcpp::shutdown();
  return 0;
}