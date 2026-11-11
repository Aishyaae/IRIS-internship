#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace cv;
using namespace std::chrono_literals;

class VisionNode : public rclcpp::Node {
public:
  // PERBAIKAN 1: Variabel array wajib diaktifkan untuk Trackbar
  int h[2] = {20, 40}, s[2] = {40, 168}, v[2] = {203, 255};


  VisionNode() : Node("vision_node") {
    // trackbar
    // namedWindow("Control");

    // createTrackbar("Hmin", "Control", &h[0], 255);
    // createTrackbar("Smin", "Control", &s[0], 255);
    // createTrackbar("Vmin", "Control", &v[0], 255);

    // createTrackbar("Hmax", "Control", &h[1], 255);
    // createTrackbar("Smax", "Control", &s[1], 255);
    // createTrackbar("Vmax", "Control", &v[1], 255);

    stream_url = declare_parameter<std::string>("stream_url", "http://admin:12345@10.110.5.153:8081/video");

    pub_centers = create_publisher<std_msgs::msg::Int32MultiArray>("/vision/lane_centers", 10);
    // pub_raw = create_publisher<sensor_msgs::msg::CompressedImage>("/camera/image_raw", 10);
    // pub_processed = create_publisher<sensor_msgs::msg::CompressedImage>("/camera/image_processed", 10);
    pub_raw = image_transport::create_publisher(this, "/camera/image_raw");
    pub_processed = image_transport::create_publisher(this, "camera/image_processed");
    
    RCLCPP_INFO(get_logger(), "Mencoba terhubung ke: %s", stream_url.c_str());

    cap.open(stream_url);
    if (!cap.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Gagal membuka stream! Cek IP HP dan koneksi Wi-Fi.");
    }

    timer = create_wall_timer(33ms, std::bind(&VisionNode::image_callback, this));
  }

private:
  sensor_msgs::msg::Image::SharedPtr matToCompressedMsg(const Mat &frame) {
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    return msg;
  }

  void image_callback() {
    if (!cap.isOpened()) {
      cap.open(stream_url);
      return;
    }
    
    Mat frame;
    cap >> frame;
    if(frame.empty()) return;
    pub_raw.publish(matToCompressedMsg(frame));

    try {
      Mat frame_bev, frame_hsv, frame_threshold, frame_morph;
      
      int width = frame.cols;
      int height = frame.rows;

      Point2f src[4] = {
        Point2f(119, 59),
        Point2f(539, 59),
        Point2f(596, 472),
        Point2f(119, 476),
      };

      Point2f dst[4] = {
        Point2f(0.0f, 0.0f), 
        Point2f(static_cast<float>(width), 0.0f), 
        Point2f(static_cast<float>(width), static_cast<float>(height)), 
        Point2f(0.0f, static_cast<float>(height)) 
      };

      Mat BEV_plot = getPerspectiveTransform(src, dst);
      warpPerspective(frame, frame_bev, BEV_plot, Size(width, height));

      cvtColor(frame_bev, frame_hsv, COLOR_BGR2HSV);
    
      inRange(frame_hsv, Scalar(h[0], s[0], v[0]), Scalar(h[1], s[1], v[1]), frame_threshold);

      frame_morph = frame_threshold.clone(); 

      erode(frame_morph, frame_morph, Mat(), Point(-1, -1), 2);
      dilate(frame_morph, frame_morph, Mat(), Point(-1, -1), 5);

      std::vector<std::vector<Point>> contours;
      findContours(frame_morph, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

      Mat frame_result = frame_bev.clone();
      std::vector<Point> centers;

      for (size_t i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);

        if (area > 400) {
          Rect boundingBox = boundingRect(contours[i]);
          
          if (boundingBox.height <= height) {
            rectangle(frame_result, boundingBox, Scalar(0, 255, 0), 2);
            Point centroid(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2);
            circle(frame_result, centroid, 5, Scalar(0, 0, 255), -1);
            centers.push_back(centroid);
          }
        }
      }

      sort(centers.begin(), centers.end(), [](const cv::Point &a, const cv::Point &b) { 
        return a.y > b.y;
      });

      std_msgs::msg::Int32MultiArray centers_msg;
      for (const auto& p : centers) {
        centers_msg.data.push_back(p.x);
        centers_msg.data.push_back(p.y);
      }
      pub_centers->publish(centers_msg);

      pub_processed.publish(matToCompressedMsg(frame_result));

      // imshow("RAW", frame);
      // imshow("Thresholding", frame_threshold);
      // imshow("BEV", frame_result);
      
      // waitKey(1);

    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "Error: %s", e.what());
    }
  }

  std::string stream_url;
  cv::VideoCapture cap;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_centers;
  // rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_raw;
  // rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_processed;

  image_transport::Publisher pub_raw;
  image_transport::Publisher pub_processed;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionNode>();

  rclcpp::spin(node);
  destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}