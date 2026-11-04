#include <memory>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace cv;
using namespace std;

class VisionNode : public rclcpp::Node {
public:
  int h[2] = {19, 92}, s[2] = {25, 98}, v[2] = {118, 246};

  VisionNode() : Node("vision_node") {

    // trackbar
    // namedWindow("Control");

    // createTrackbar("Hmin", "Control", &h[0], 255);
    // createTrackbar("Smin", "Control", &s[0], 255);
    // createTrackbar("Vmin", "Control", &v[0], 255);

    // createTrackbar("Hmax", "Control", &h[1], 255);
    // createTrackbar("Smax", "Control", &s[1], 255);
    // createTrackbar("Vmax", "Control", &v[1], 255);


    // quality of Service (QoS) disesuaikan agar cocok dengan publisher gazebo camera
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    // publisher untuk kirim centers ke master
    pub_centers = create_publisher<std_msgs::msg::Int32MultiArray>("/vision/lane_centers", 10);

    // publisher ke basestation
    pub_raw = create_publisher<sensor_msgs::msg::CompressedImage>("/camera/image_raw", 10);
    pub_processed = create_publisher<sensor_msgs::msg::CompressedImage>("/camera/image_processed", 10);
    
    // subscriber topic kamera depan
    sub_image = create_subscription<sensor_msgs::msg::Image>(
      "/catvehicle/camera_front/image_raw_front",
      qos_profile,
      std::bind(&VisionNode::image_callback, this, std::placeholders::_1)
    );
  }

private:
  // tipe pesan sensor_msgs/CompressedImage
  sensor_msgs::msg::CompressedImage matToCompressedMsg(const Mat &frame) {
    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = this->now();
    msg.format = "jpeg";
    std::vector<uchar> buffer;
    imencode(".jpg", frame, buffer);
    msg.data = buffer;
    return msg;
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    try {
      // konversi dari ROS image ke opencv (brg8)
      Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

      Mat frame_bev, frame_hsv, frame_threshold, frame_output;
      
      // bev
      int width = frame.cols;
      int height = frame.rows;

      Point2f src[4] = {
        Point2f(316, 460),
        Point2f(646, 472),
        Point2f(793, 616),
        Point2f(11, 537),
      };

      Point2f dst[4] = {
        Point2f(0.0f, 0.0f), // Kiri atas
        Point2f(static_cast<float>(width), 0.0f), // Kanan atas
        Point2f(static_cast<float>(width), static_cast<float>(height)), // kanan bawah
        Point2f(0.0f, static_cast<float>(height)) // Kiri bawah
      };

      Mat BEV_plot = getPerspectiveTransform(src, dst);
      warpPerspective(frame, frame_bev, BEV_plot, Size(width, height));

      cvtColor(frame_bev, frame_hsv, COLOR_BGR2HSV);
    
      // threshold hsv
      inRange(frame_hsv, Scalar(h[0], s[0], v[0]), Scalar(h[1], s[1], v[1]), frame_threshold);

      // contour
      vector<vector<Point>> contours;
      findContours(frame_threshold, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

      Mat frame_result = frame_bev.clone();
      vector<Point> centers;

      int margin = 40; // abaikan kontur dalam jarak 40px dari tepi layar

      for (size_t i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);

        // filter luas dan batas tepi
        if (area > 400 && area < 8000) {
          Rect boundingBox = boundingRect(contours[i]);

          bool isNotTouchingEdge = (boundingBox.x > margin) && (boundingBox.x + boundingBox.width < (width - margin));
          
          if (boundingBox.height < (height * 0.5) && isNotTouchingEdge) {
            rectangle(frame_result, boundingBox, Scalar(0, 255, 0), 2);

            Point centroid(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2);
            circle(frame_result, centroid, 5, Scalar(0, 0, 255), -1);

            centers.push_back(centroid);
          }
        }
      }

      // urutkan titik dari yang paling dekat mobil
      sort(
        centers.begin(), 
        centers.end(), 
        [](const cv::Point &a, const cv::Point &b) { return a.y > b.y;
        }
      );

      // publish centers_msg untuk master
      std_msgs::msg::Int32MultiArray centers_msg;
      for (const auto& p : centers) {
        centers_msg.data.push_back(p.x);
        centers_msg.data.push_back(p.y);
      }
      pub_centers->publish(centers_msg);

      // publish gambar untuk basestation
      pub_raw->publish(matToCompressedMsg(frame));
      pub_processed->publish(matToCompressedMsg(frame_result));

      // imshow("RAW", frame);
      // imshow("Thresholding", frame_threshold);
      imshow("BEV", frame_result);
      
      waitKey(1);

    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "Gagal mengonversi gambar dengan cv_bridge: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_centers;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_raw;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_processed;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = make_shared<VisionNode>();

  rclcpp::spin(node);
  destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}