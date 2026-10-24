#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

class objectTracker {
private:
    double fx, fy, cx, cy;

public:
    objectTracker(const std::string& yamlPath) : fx(0), fy(0), cx(0), cy(0) { // 
        cv::FileStorage fs(yamlPath, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            std::cerr << "Error: File 'yaml/kalibrasi.yaml' tidak ditemukan!" << std::endl;
            exit(-1);
        }

        // ambil nilai
        fs["fx"] >> fx;
        fs["fy"] >> fy;
        fs["cx"] >> cx;
        fs["cy"] >> cy;
        fs.release(); // tutup file

        std::cout << "Berhasil memuat kalibrasi!" << std::endl;
        std::cout << "fx: " << fx << " | fy: " << fy << std::endl;
        std::cout << "cx: " << cx << " | cy: " << cy << std::endl;
    }

    // pengolahan citra dan tracker
    void trackObjects(const cv::Mat& hlsImage, cv::Mat& resultFrame, const std::string& name, double wReal, cv::Scalar lowerHLS, cv::Scalar upperHLS, cv::Scalar boxColor) {
        // thresholding
        cv::Mat mask;
        cv::inRange(hlsImage, lowerHLS, upperHLS, mask);

        // morfologi (erode dilate)
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        // contour
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // cari contour terbesar
        int bestIdx = -1;
        double maxArea = 1000.0; // Hanya kontur > 1000 yang diproses

        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > maxArea) {
                maxArea = area;
                bestIdx = static_cast<int>(i);
            }
        }

        // buat bounding box jika contour valid
        if (bestIdx != -1) {
            cv::Rect boundingBox = cv::boundingRect(contours[bestIdx]);

            // wPixel (lebar), xPixel yPixel (titik pusat objek)
            double wPixel = boundingBox.width;
            double xPixel = boundingBox.x + boundingBox.width / 2.0;                double yPixel = boundingBox.y + boundingBox.height / 2.0;

            // estimasi koordinat x, y, z dengan pinhole camera model
            double z = (fx * wReal) / wPixel;
            double x = ((xPixel - cx) * z) / fx;
            double y = ((yPixel - cy) * z) / fy;

            // bounding box dan centroid
            cv::rectangle(resultFrame, boundingBox, boxColor, 2);
            cv::circle(resultFrame, cv::Point(xPixel, yPixel), 4, boxColor, -1);

            // format teks
            std::string distance = cv::format("Z : %.1f cm", z);
            std::string labelXYZ = cv::format("3D: (%.1f, %.1f, %.1f) cm", x, y, z);

            // buat background transparan
            int bgHeight = 45; // tinggi kotak
            int bgWidth  = std::max(130, boundingBox.width);

            // koordinat kotak background tepat di atas bounding box
            cv::Rect bgRect(boundingBox.x, boundingBox.y - bgHeight, bgWidth, bgHeight);

            if (bgRect.y >= 0 && bgRect.x + bgRect.width <= resultFrame.cols) {
                cv::Mat roi = resultFrame(bgRect);
                cv::Mat colorBg(roi.size(), roi.type(), cv::Scalar(0, 0, 0)); // Warna hitam
                    
                // Transparansi 60% hitam
                double alpha = 0.6;
                cv::addWeighted(colorBg, alpha, roi, 1.0 - alpha, 0.0, roi);
            }

            // cetak nama
            cv::putText(resultFrame, name, cv::Point(boundingBox.x + 5, boundingBox.y - 30), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

            // cetak jarak
            cv::putText(resultFrame, distance, cv::Point(boundingBox.x + 5, boundingBox.y - 17), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

            // cetak koordinat x,y,z
            cv::putText(resultFrame, labelXYZ, cv::Point(boundingBox.x + 5, boundingBox.y - 4), cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
                
        }
    
    }
};

int main() {
    // baca file yaml
    objectTracker tracker("yaml/kalibrasi.yaml");

    // koneksi IP webcam 
	std::string streamAddress = "http://xxx.xxx.xxx.xxx:8080/video"; 
    cv::VideoCapture cap(streamAddress, cv::CAP_FFMPEG);
    
    if (!cap.isOpened()) {
        std::cerr << "Error: gagal membuka webcam" << std::endl;
        return -1;
    }

    // ukuran benda asli (cm)
    double wRealYellow = 7.2;
    double wRealRed = 5.0;
    double wRealBlue = 5.0;

    cv::Mat frame;

    // cv::namedWindow("Morph", cv::WINDOW_NORMAL);
    cv::namedWindow("Result", cv::WINDOW_NORMAL);

    // Variabel untuk perhitungan FPS
    double fpsSmooth = 0.0;
    double alpha = 0.1;

    while (true) {
        int64 tStart = cv::getTickCount(); // catat waktu awal loop

        cap.grab();
        cap >> frame;
        if (frame.empty()) break;

        // convert color
        cv::Mat hlsImage;
        cv::cvtColor(frame, hlsImage, cv::COLOR_BGR2HLS);

        cv::Mat result = frame.clone(); // clone untuk hasil

        // petunjuk untuk keluar
        std::string infoText = "Tekan 'q' untuk keluar";
        cv::putText(frame, infoText, cv::Point(120, 30),cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 1);

        // objek kuning
        tracker.trackObjects(hlsImage, result, "Spons Kuning", wRealYellow, cv::Scalar(22, 4, 89), cv::Scalar(36, 195, 255), cv::Scalar(0, 255, 255));

        // objek merah
        tracker.trackObjects(hlsImage, result, "Rubik Merah",  wRealRed, cv::Scalar(0, 57, 70), cv::Scalar(10, 172, 255), cv::Scalar(0, 0, 255));

        // objek biru
        tracker.trackObjects(hlsImage, result, "Tutup Biru",   wRealBlue, cv::Scalar(96, 73, 21), cv::Scalar(128, 176, 255), cv::Scalar(255, 0, 0));

        // catat waktu akhir lalu hitung fps
        int64 tEnd = cv::getTickCount();
        double timeSec = (tEnd - tStart) / cv::getTickFrequency();
        
        if (timeSec > 0) {
            double fpsRaw = 1.0 / timeSec;
            
            // inisialisasi awal jika fpsSmooth masih 0
            if (fpsSmooth == 0.0) {
                fpsSmooth = fpsRaw;
            } else {
                // perhalus angka FPS agar tidak loncat-loncat
                fpsSmooth = (alpha * fpsRaw) + ((1.0 - alpha) * fpsSmooth);
            }
        }

        // tampilkan fps
        std::string fpsText = cv::format("FPS: %.1f", fpsSmooth);
        cv::putText(result, fpsText, cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

        // tampilkan hasil
        cv::imshow("Result", result);

        int key = cv::waitKey(25);
        if(key == 'q') break; // press q untuk keluar jendela
    }

    cap.release();
    cv::destroyAllWindows();
	return 0;
} 
