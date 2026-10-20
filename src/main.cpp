#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

struct Object {
    std::string name;
    float wReal;
    cv::Scalar lowerHLS;
    cv::Scalar upperHLS;
    cv::Scalar boxColor;
}

int main() {
    cv::FileStorage fs("yaml/kalibrasi.yaml", cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Error: File 'yaml/kalibrasi.yaml' tidak ditemukan! Jalankan ./calib dulu." << std::endl;
        return -1;
    }

    cv::Mat K;
    fs["camera_matrix"] >> K;
    fs.release();
    float fx = static_cast<float>(K.at<double>(0, 0));
    float fy = static_cast<float>(K.at<double>(1, 1));
    float cx = static_cast<float>(K.at<double>(0, 2));
    float cy = static_cast<float>(K.at<double>(1, 2));

    // koneksi webcam 
	std::string streamAddress = "IP"; 
    cv::VideoCapture cap(streamAddress, cv::CAP_FFMPEG);
    
    if (!cap.isOpened()) {
        std::cerr << "Error: gagal membuka webcam" << std::endl;
        return -1;
    }

    std::vector<Object> targets = {
        {"Yellow", 7.0f, cv::Scalar(22, 4, 89),   cv::Scalar(36, 195, 255), cv::Scalar(0, 255, 255)},
        {"Red", 5.0f, cv::Scalar(0, 0, 70),    cv::Scalar(10, 140, 255), cv::Scalar(0, 0, 255)},
        {"Blue", 5.0f, cv::Scalar(96, 73, 50),  cv::Scalar(128, 176, 255),cv::Scalar(255, 0, 0)}
    };

    cv::Mat frame;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // 1. Convert Color
        cv::Mat hlsImage;
        cv::cvtColor(frame, hlsImage, cv::COLOR_BGR2HLS);

        // cv::namedWindow("Yellow", cv::WINDOW_NORMAL);
        // cv::namedWindow("Red", cv::WINDOW_NORMAL);
        // cv::namedWindow("Blue", cv::WINDOW_NORMAL);
        cv::namedWindow("All", cv::WINDOW_NORMAL);
        cv::namedWindow("Morph", cv::WINDOW_NORMAL);
        cv::namedWindow("Result", cv::WINDOW_NORMAL);

        // 2. Thresholding
        cv::Mat yellow_threshold;
        cv::inRange(
            hlsImage,
            cv::Scalar(22, 4, 89),
            cv::Scalar(36, 195, 255),
            yellow_threshold
        );
        cv::Mat red_threshold;
        cv::inRange(
            hlsImage,
            cv::Scalar(0, 0, 70),
            cv::Scalar(10, 140, 255),
            red_threshold
        );
        cv::Mat blue_threshold;
        cv::inRange(
            hlsImage,
            cv::Scalar(96, 73, 50),
            cv::Scalar(128, 176, 255),
            blue_threshold
        );
        cv::Mat combined;
        cv::bitwise_or(yellow_threshold, red_threshold, combined);
        cv::bitwise_or(combined, blue_threshold, combined);

        // 3. Morphological Erode & Dilate
        cv::Mat morphImage = combined.clone();
        // Function Morphological Erode & Dilate
        cv::erode(morphImage, morphImage, cv::Mat(), cv::Point(-1, -1), 10);
        cv::dilate(morphImage, morphImage, cv::Mat(), cv::Point(-1, -1), 13);
        cv::erode(morphImage, morphImage, cv::Mat(), cv::Point(-1, -1), 8);

        // 4. Contour Detection
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(morphImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 5. Bounding Box
        cv::Mat result = frame.clone();
        for (size_t i = 0; i < contours.size(); i++) {
            if(cv::contourArea(contours[i]) > 1000) { // Filter area
                cv::Rect boundingBox = cv::boundingRect(contours[i]);
                cv::rectangle(result, boundingBox, cv::Scalar(0, 255, 0), 2);
            }
        }

        // cv::imshow("Yellow", yellow_threshold);
        // cv::imshow("Red", red_threshold);
        // cv::imshow("Blue", blue_threshold);
        cv::imshow("All", combined);
        cv::imshow("Morph", morphImage);
        cv::imshow("Result", result);

        int key = cv::waitKey(25);
        if(key == 'q') break; // menunda program selama 25 milidetik untuk GUI gambar frame baru. kalo q break
    }

    cap.release();
    cv::destroyAllWindows();
	return 0;
} 