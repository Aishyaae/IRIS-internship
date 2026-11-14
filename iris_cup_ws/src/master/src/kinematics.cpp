
#include <memory>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <vector>
#include <iostream>
#include <cstdint>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std;

const double CPR = 410.0;
const double wheels_diameter = 6.7;
const double wheels_around = M_PI * wheels_diameter;
const double cm_per_tick = wheels_around / CPR; // 0.05 cm/tick
const double delta_s = 0.02; // 20 ms

const int steer_offset = 85;

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

    pid_timer = this->create_wall_timer(std::chrono::milliseconds(20), bind(&MasterNode::pid_callback, this)); // 20ms

    RCLCPP_INFO(get_logger(), "Master Node Kinematika Dimulai");
  }

private:
  void pixel_convertion(int target_x, int target_y, double &dx, double &dy) {
    int center_x = image_width / 2;
    int bottom_y = image_height;

    double xm_per_pix = 0.000838;
    double ym_per_pix = 0.000388;
    // double dummy_scale = 0.01;

    dx = (target_x - center_x) * xm_per_pix;
    dy = (bottom_y - target_y) * ym_per_pix;
  }

  bool select_lookahead_point(const std::vector<int32_t>& points,int& target_x, int& target_y, double& dx, double& dy) {
    const double lookahead_distance = 0.15; // 15 cm

    double min_distance = INFINITY;
    bool found = false;

    // periksa titik dari yang terdekat
    for (size_t i = 0; i + 1 < points.size(); i += 2) {
        int candidate_x = points[i];
        int candidate_y = points[i + 1];

        double candidate_dx = 0.0;
        double candidate_dy = 0.0;

        // konversi pixel ke meter
        pixel_convertion(
            candidate_x,
            candidate_y,
            candidate_dx,
            candidate_dy
        );

        // abaikan titik yang tidak berada di depan robot
        if (candidate_dy <= 0.0) continue;

        // hitung jarak aktual dari robot
        double candidate_Ld = hypot(candidate_dx, candidate_dy);

        // RCLCPP_INFO(
        //     get_logger(),
        //     "Candidate (%d,%d) | Ld: %.3f m",
        //     candidate_x,
        //     candidate_y,
        //     candidate_Ld
        // );

        // periksa apakah mencapai lookahead distance
        if (candidate_Ld >= lookahead_distance && candidate_Ld < min_distance) {
          min_distance = candidate_Ld;

          target_x = candidate_x;
          target_y = candidate_y;

          dx = candidate_dx;
          dy = candidate_dy;

          found = true;
        }
    }
    return found;
  }

  double pure_pursuit(double dx, double dy) {
    double Ld = hypot(dx, dy);

    if (Ld < 0.001) return 0.0; // cegah pembagian dengan 0

    // sudut arah titik target terhadap robot
    double theta = atan2(dx, dy);
    // curvature
    double k = (2.0 * sin(theta) / Ld);
    // sudut steering ackermann
    double virtual_steering = atan(wheel_base * k);
    // ubah ke derajat
    return virtual_steering * (180.0 / M_PI);
  }

  int vel_pid(int actual_vel) {
    rclcpp::Time current_time = this->now();

    double dt = 0.02;
    if (!first_pid_call) {
      dt = (current_time - last_pid_time).seconds();
    }

    last_pid_time = current_time;
    first_pid_call = false;

    if (dt <= 0.0) { // hidari perhitungan dengan dt tidak valid
      return current_pwm;
    }

    double error = target_vel - actual_vel;

    // proportional
    double p = kp * error;
    
    // integral
    integral_vel_error += error * dt;
    integral_vel_error = clamp(integral_vel_error, -500.0, 500.0);
    double i = ki * integral_vel_error;

    // derivative
    double d_error = (error - last_vel_error) / dt;
    double d = kd * d_error;

    int correction = static_cast<int>(p + i + d);

    current_pwm += correction;
    current_pwm = clamp(current_pwm, -9999, 9999);

    last_vel_error = error;
    return current_pwm;
  }

  void feedback_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 12) return;

    int encoder_delta = msg->data[2];

    // konversi ke kecepatan
    double vel_cms = (encoder_delta * cm_per_tick) / delta_s;
    actual_vel = static_cast<int>(vel_cms);

    std_msgs::msg::Float32 speed_msg;
    speed_msg.data = static_cast<float>(actual_vel);
    pub_speed_web->publish(speed_msg);
  }

  void vision_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    // int steer = last_steer;

    // jika vision tidak mendeteksi garis
    if (msg->data.empty()) {
      target_valid = false;
      RCLCPP_WARN(this->get_logger(), "Vision data kosong!");
      // steer= last_steer; // pertahankan sudut terakhir

      // // kurangi kecepatan
      // current_pwm = max(0, current_pwm - 20);
      // pwm = current_pwm;
    } 
    else 
    {
      int target_x = 0;
      int target_y = 0;

      double dx = 0.0;
      double dy = 0.0;

      // cari titik target lookahead
      bool target_found = select_lookahead_point(msg->data, target_x, target_y, dx, dy);

      target_valid = target_found;

      if (target_found) {
        // pure pursuit
        int pure_steer = static_cast<int>(pure_pursuit(dx, dy));

        
        last_steer = steer_offset + pure_steer;
        last_steer = clamp(last_steer, steer_offset - 30, steer_offset + 30);
      } 
      // else {
      //   RCLCPP_WARN(get_logger(), "Lookahead point tidak ditemukan!");

      //   pwm = 0;
      // }

      cout << "\rX: " << target_x << " Y: " << target_y << " dx: " << dx << " dy: " << dy << " Last steer: " << last_steer << flush;
    }
  }

    void pid_callback() {
      int pwm = 0;

      if (target_valid) {
        pwm = 3500;
        // pwm = vel_pid(actual_vel);
        // double steer_angle = last_steer - steer_offset;
        // double angle_rad = abs(steer_angle) * M_PI / 180.0;
        // pwm = static_cast<int>(3500 * cos(angle_rad) * 0.8);

      } else {
        pwm = 3500;
        // current_pwm -= 200; // perlambatan bertahap
        // if (current_pwm < 0) {
        //     current_pwm = 0;
        //     integral_vel_error = 0.0;
        //     last_vel_error = 0.0;
        // }
        // pwm = current_pwm;
      }

      // publish nilai ke udp sender
      std_msgs::msg::Int32MultiArray cmd_msg;
      cmd_msg.data = {pwm, last_steer};
      pub_cmd->publish(cmd_msg);    

      // RCLCPP_INFO(
      //     get_logger(),
      //     "TargetX: %d | TargetY: %d | dx: %.4f | dy: %.4f | Steering: %d",
      //     target_x, target_y, dx, dy, steer
      // );

      // publish steering angle ke basestation
      std_msgs::msg::Float32 steering_web_msg;
      steering_web_msg.data = static_cast<float>(last_steer);
      pub_steering_web->publish(steering_web_msg);

      cout << "\rpwm: " << pwm << flush;
    }

  double wheel_base;
  int image_width, image_height;

  int target_vel = 30;
  int actual_vel = 0;
  int current_pwm = 1500;

  double kp, ki, kd;
  double integral_vel_error = 0.0;
  double last_vel_error = 0.0;
  int last_steer = steer_offset;

  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_cmd;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_steering_web;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_speed_web;

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_vision;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_feedback;

  rclcpp::TimerBase::SharedPtr pid_timer;
  rclcpp::Time last_pid_time;
  bool first_pid_call = true;
  bool target_valid = false;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<MasterNode>());
  rclcpp::shutdown();
  return 0;
}