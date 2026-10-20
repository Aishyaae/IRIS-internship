#include <opencv2/opencv.hpp> //fungsi pemrosesan gambar
#include <iostream>
#include <vector>

int main () {
    std::string streamAddress = "IP"; 
    cv::VideoCapture cap(streamAddress, cv::CAP_FFMPEG); 
    cv::namedWindow("Kamera", cv::WINDOW_NORMAL);
    
    if (!cap.isOpened()) {
        std::cerr << "Error: Tidak dapat membuka IP Webcam di " << streamAddress << std::endl;
        return -1;
    }

    cv::Mat frame, gray; // kontainer matriks untuk simpan gata gamber dari setiap frame video nanti

    cv::Size patternSize(9, 6);
    float squareSize = 0.9f;

    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;

    std::vector<cv::Point3f> obj;
    for (int i = 0; i < patternSize.height; i++) {
        for (int j = 0; j < patternSize.width; j++) {
            obj.push_back(cv::Point3f(j * squareSize, i * squareSize, 0.0f));
        }
    }

    int targetFrame = 15;

    //perulangan
    while (cap.isOpened()) { // selama koneksi ke kamera berhasil dibuka dan aktif
        cap >> frame; // ambil 1 frame gambar terbaru dari video cap, lalu simpan ke variabel frame
        if (frame.empty()) break;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool result = cv::findChessboardCorners(gray, patternSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        if(result) {
            cv::drawChessboardCorners(frame, patternSize, corners, true);
        }

        std::string infoText = "Tersimpan: " + std::to_string(imagePoints.size()) + "/" + std::to_string(targetFrame);
        cv::putText(frame, infoText, cv::Point(20, 40),cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 1);

        cv::imshow("Kamera", frame); // tampilkan variabel frame ke jendela GUI judul jendelanya "Kamera"

        int key = cv::waitKey(25);
        if(key == 'q') { // menunda program selama 25 milidetik untuk GUI gambar frame baru. kalo q break
            break;
        } else if (key == ' ' && result) {
            cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1);
            cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            objectPoints.push_back(obj);
            imagePoints.push_back(corners);
        }

        if (imagePoints.size() == targetFrame) {
            break;
        }
    }

    if(!imagePoints.empty()) {
        cv::Mat cameraMatrix, distCoeffs, rvecs, tvecs;
        double rms = cv::calibrateCamera(objectPoints, imagePoints, frame.size(), cameraMatrix, distCoeffs, rvecs, tvecs);

        std::cout << "RMS " << rms << std::endl;
        std::cout << "Matriks kamera " << cameraMatrix << std::endl;

        cv::FileStorage fs("yaml/kalibrasi.yaml", cv::FileStorage::WRITE);
        fs << "camera_matrix" << cameraMatrix;
        fs << "distortion_coefficients" << distCoeffs;
        fs.release();
    }
    return 0;
    
}