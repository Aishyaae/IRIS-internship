#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>

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
  int h[2] = {20, 62}, s[2] = {20, 255}, v[2] = {209, 255};
  // int src_tl_x = 344, src_tl_y = 444;
  // int src_tr_x = 656, src_tr_y = 450;
  // int src_br_x = 795, src_br_y = 597;
  // int src_bl_x = 9,   src_bl_y = 527;
  // int frame_width = 640;
  // int frame_height = 480;

  VisionNode() : Node("vision_node") {
    // trackbar
    // namedWindow("Control");
    // createTrackbar("Hmin", "Control", &h[0], 255);
    // createTrackbar("Smin", "Control", &s[0], 255);
    // createTrackbar("Vmin", "Control", &v[0], 255);
    // createTrackbar("Hmax", "Control", &h[1], 255);
    // createTrackbar("Smax", "Control", &s[1], 255);
    // createTrackbar("Vmax", "Control", &v[1], 255);

    // namedWindow("Control BEV");
    // createTrackbar("TL X", "Control BEV", &src_tl_x, frame_width - 1);
    // createTrackbar("TL Y", "Control BEV", &src_tl_y, frame_height - 1);
    // createTrackbar("TR X", "Control BEV", &src_tr_x, frame_width - 1);
    // createTrackbar("TR Y", "Control BEV", &src_tr_y, frame_height - 1);
    // createTrackbar("BR X", "Control BEV", &src_br_x, frame_width - 1);
    // createTrackbar("BR Y", "Control BEV", &src_br_y, frame_height - 1);
    // createTrackbar("BL X", "Control BEV", &src_bl_x, frame_width - 1);
    // createTrackbar("BL Y", "Control BEV", &src_bl_y, frame_height - 1);


    stream_url = declare_parameter<std::string>("stream_url", "http://admin:12345@192.168.54.247:8081/video");
    calib_file = declare_parameter<std::string>("calib_file", "src/vision/src/kalibrasi.yaml");
    show_gui = declare_parameter<bool>("show_gui", true);
    jpeg_quality = declare_parameter<int>("jpeg_quality", 70);

    pub_centers = create_publisher<std_msgs::msg::Int32MultiArray>("/vision/lane_centers", 10);
    pub_raw = create_publisher<sensor_msgs::msg::CompressedImage>("/camera/image_raw", 10);
    pub_processed = create_publisher<sensor_msgs::msg::CompressedImage>("/camera/image_processed", 10);
    pub_bev = create_publisher<sensor_msgs::msg::CompressedImage>("/camera/image_bev", 10);

    // pub_raw = image_transport::create_publisher(this, "/camera/image_raw");
    // pub_processed = image_transport::create_publisher(this, "camera/image_processed");

    RCLCPP_INFO(get_logger(), "Mencoba terhubung ke: %s", stream_url.c_str());

    cap.open(stream_url);
    // cap.open(2);
    if (!cap.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Gagal membuka stream! Cek IP HP dan koneksi Wi-Fi.");
    }

    loadCalibrationData();

    // Kernel morfologi dibuat sekali saja
    kernel_open = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    kernel_close = getStructuringElement(MORPH_RECT, Size(21, 21));

    // Thread capture: terus membaca stream dan hanya menyimpan frame terbaru,
    // supaya buffer FFmpeg tidak menumpuk (penyebab delay makin lama makin besar).
    grab_thread = std::thread(&VisionNode::grabLoop, this);

    timer = create_wall_timer(33ms, std::bind(&VisionNode::image_callback, this));
  }

  ~VisionNode() override {
    running = false;
    if (grab_thread.joinable()) grab_thread.join();
  }

private:
  sensor_msgs::msg::CompressedImage matToCompressedMsg(const Mat &frame) {
      sensor_msgs::msg::CompressedImage msg;
      msg.header.stamp = this->now();
      msg.format = "jpeg";
      std::vector<uchar> buffer;

      imencode(".jpg", frame, buffer, {IMWRITE_JPEG_QUALITY, jpeg_quality});
      msg.data = buffer;
      return msg;
    }

  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;
  bool isCalibrated = false;

  // Map undistort & matriks BEV di-cache, dibuat ulang hanya jika ukuran frame berubah
  cv::Mat undist_map1, undist_map2;
  cv::Size undist_size;
  cv::Mat BEV_plot;
  cv::Size bev_size;
  cv::Mat kernel_open, kernel_close;

  // Fungsi untuk memuat file YAML
  void loadCalibrationData() {
    FileStorage fs(calib_file, FileStorage::READ);
    if (fs.isOpened()) {
      fs["camera_matrix"] >> cameraMatrix;
      fs["distortion_coefficients"] >> distCoeffs;
      fs.release();
      isCalibrated = true;
      RCLCPP_INFO(get_logger(), "File kalibrasi berhasil dimuat.");
    } else {
      RCLCPP_WARN(get_logger(), "File kalibrasi tidak ditemukan, menggunakan frame mentah.");
    }
  }

  void grabLoop() {
    Mat f;
    while (running) {
      if (!cap.isOpened()) {
        cap.open(stream_url);
        if (!cap.isOpened()) {
          std::this_thread::sleep_for(500ms);
          continue;
        }
      }
      if (!cap.read(f) || f.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Gagal membaca frame, mencoba ulang...");
        cap.release();
        std::this_thread::sleep_for(200ms);
        continue;
      }
      std::lock_guard<std::mutex> lk(frame_mtx);
      f.copyTo(latest_frame);
      has_new_frame = true;
    }
  }

  void image_callback() {
    Mat frame, raw_frame;
    {
      std::lock_guard<std::mutex> lk(frame_mtx);
      if (!has_new_frame) return;  // jangan proses frame yang sama dua kali
      latest_frame.copyTo(raw_frame);
      has_new_frame = false;
    }

    if (isCalibrated) {
      if (raw_frame.size() != undist_size) {
        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix, raw_frame.size(), CV_16SC2, undist_map1, undist_map2);
        undist_size = raw_frame.size();
      }
      remap(raw_frame, frame, undist_map1, undist_map2, INTER_LINEAR);
    } else {
      frame = raw_frame;
    }

    // rotate(frame, frame, ROTATE_90_COUNTERCLOCKWISE);

    if (pub_raw->get_subscription_count() > 0) {
      pub_raw->publish(matToCompressedMsg(frame));
    }

    try {
      Mat frame_bev, frame_hsv, frame_threshold, frame_morph;

      int width = frame.cols;
      int height = frame.rows;

      if (frame.size() != bev_size) {
        Point2f src[4] = {
          Point2f(177, 82), // Top Left
          Point2f(498, 70), // Top Right
          Point2f(625, 479), // Bottom Right
          Point2f(65, 462)  // Bottom Left
        };

        Point2f dst[4] = {
          Point2f(0, 0),
          Point2f(width, 0),
          Point2f(width, height),
          Point2f(0, height)
        };

        BEV_plot = getPerspectiveTransform(src, dst);
        bev_size = frame.size();
      }
      warpPerspective(frame, frame_bev, BEV_plot, Size(width, height));

      cvtColor(frame_bev, frame_hsv, COLOR_BGR2HSV);

      inRange(frame_hsv, Scalar(h[0], s[0], v[0]), Scalar(h[1], s[1], v[1]), frame_threshold);

      // morph closing
      morphologyEx(frame_threshold, frame_morph, MORPH_OPEN, kernel_open, Point(-1, -1), 1);
      morphologyEx(frame_morph, frame_morph, MORPH_CLOSE, kernel_close, Point(-1, -1), 1);


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

      sort(
        centers.begin(),
        centers.end(),
        [](const cv::Point &a, const cv::Point &b) {
        return a.y > b.y;
      });

      std_msgs::msg::Int32MultiArray centers_msg;
      for (const auto& p : centers) {
        centers_msg.data.push_back(p.x);
        centers_msg.data.push_back(p.y);
      }
      pub_centers->publish(centers_msg);

      if (!centers.empty()) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Vision -> Target Titik: X=%d, Y=%d", centers[0].x, centers[0].y);
      }

      if (pub_bev->get_subscription_count() > 0) {
        pub_bev->publish(matToCompressedMsg(frame_result));
      }
      if (pub_processed->get_subscription_count() > 0) {
        pub_processed->publish(matToCompressedMsg(frame_threshold));
      }

      if (show_gui) {
        imshow("RAW", frame);
        imshow("Thresholding", frame_threshold);
        imshow("BEV", frame_result);

        waitKey(1);
      }

    } catch (const cv::Exception &e) {
      RCLCPP_ERROR(get_logger(), "Error: %s", e.what());
    }
  }

  std::string stream_url;
  std::string calib_file;
  bool show_gui = true;
  int jpeg_quality = 70;
  cv::VideoCapture cap;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_centers;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_raw;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_processed;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_bev;

  // Capture thread
  std::thread grab_thread;
  std::mutex frame_mtx;
  cv::Mat latest_frame;
  bool has_new_frame = false;
  std::atomic<bool> running{true};

  // image_transport::Publisher pub_raw;
  // image_transport::Publisher pub_processed;
};

int main(int argc, char **argv) {
  // matikan Log Spam FFmpeg
  setenv("OPENCV_FFMPEG_LOGLEVEL", "-8", 1);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionNode>();

  rclcpp::spin(node);
  destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}