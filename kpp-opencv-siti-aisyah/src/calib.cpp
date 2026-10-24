#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

int main () {
    // koneksi IP webcam
    std::string streamAddress = "http://192.168.110.127:8080/video"; 
    cv::VideoCapture cap(streamAddress, cv::CAP_FFMPEG); 

    cv::namedWindow("Kamera", cv::WINDOW_NORMAL);
    
    if (!cap.isOpened()) { // cek apakah kamera terkoneksi
        std::cerr << "Error: Tidak dapat membuka IP Webcam di " << streamAddress << std::endl;
        return -1;
    }

    cv::Mat frame, gray; // frame nyimpen gambar BGR asli, gray simpan gambar versi abu-abu

    // jumlah sudut papan dan ukuran kotak aslinya
    cv::Size patternSize(9, 6);
    float squareSize = 2.5f;

    std::vector<std::vector<cv::Point2f>> imagePoints; // koordinat 2D pixel sudut dari setiap frame
    std::vector<std::vector<cv::Point3f>> objectPoints; // koordinat 3D fisik nyata dari sudut

    // membuat pola koordinat 3D spasial ideal papan berdasarkan kelipatan squareSize
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < patternSize.height; i++) {
        for (int j = 0; j < patternSize.width; j++) {
            obj.push_back(cv::Point3f(j * squareSize, i * squareSize, 0.0f));
        }
    }

    int targetFrame = 15; // jumlah sampel gambar
    cv::Size imageSize; // variabel resolusi gambar

    while (cap.isOpened()) { // selama koneksi ke kamera berhasil dibuka dan aktif
        cap.grab(); // buang frame lama yg tertahan di antrean buffer
        cap >> frame; // ambil 1 frame gambar terbaru dari video cap, lalu simpan ke variabel frame
        if (frame.empty()) break;

        imageSize = frame.size(); // dimensi lebar dan tinggi gambar

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY); // konversi warna frame dari BGR ke gray

        std::vector<cv::Point2f> corners; // koordinat pixel titik sudut

        // cari titik sudut internal papan dan true kalau seluruhnya berhasil ditemukan
        bool result = cv::findChessboardCorners(gray, patternSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        if(result) { // jika terdeteksi gambar pola
            cv::drawChessboardCorners(frame, patternSize, corners, true);
        }

        // petunjuk tombol ambil frame
        std::string infoText1 = "Tekan 'space' untuk menyimpan frame";
        cv::putText(frame, infoText1, cv::Point(20, 40),cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 1);

        // petunjuk jumlah sampel
        std::string infoText2 = "Tersimpan: " + std::to_string(imagePoints.size()) + "/" + std::to_string(targetFrame);
        cv::putText(frame, infoText2, cv::Point(20, 75),cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 1);

        cv::imshow("Kamera", frame); // tampilkan variabel frame ke jendela GUI judul jendelanya "Kamera"

        int key = cv::waitKey(25); //menunda program selama 25 milidetik
        if(key == 'q') { // kalo q break
            break;
        } else if (key == ' ' && result) { // kalo space dan papan terdeteksi, simpan
            cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1);
            cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            objectPoints.push_back(obj);
            imagePoints.push_back(corners);
        }

        // hentikan perulangan kalo udah capai target
        if (imagePoints.size() == targetFrame) {
            std::cout << "Sampel cukup!" << std::endl;
            break;
        }
    }

    if(!imagePoints.empty()) {
        cv::Mat cameraMatrix, distCoeffs, rvecs, tvecs;
        // kalibrasi kamera
        double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);

        // ambil parameter yang dibutuhkan
        double fx = cameraMatrix.at<double>(0, 0);
        double fy = cameraMatrix.at<double>(1, 1);
        double cx = cameraMatrix.at<double>(0, 2);
        double cy = cameraMatrix.at<double>(1, 2);

        std::cout << "RMS " << rms << std::endl;
        std::cout << "fx: " << fx << " | fy: " << fy << std::endl;
        std::cout << "cx: " << cx << " | cy: " << cy << std::endl;

        cv::FileStorage fs("yaml/kalibrasi.yaml", cv::FileStorage::WRITE); // buka yaml mode tulis

        // simpan parameter
        if (fs.isOpened()) {
            fs << "camera_matrix" << cameraMatrix;
            fs << "distortion_coefficients" << distCoeffs;
            fs << "fx" << fx;
            fs << "fy" << fy;
            fs << "cx" << cx;
            fs << "cy" << cy;
            fs << "imageWidth" << imageSize.width;
            fs << "imageHeight" << imageSize.height;
            fs << "rms" << rms;

            fs.release(); // tutup file
            std::cout << "\nParameter berhasil disimpan" << std::endl;
        } else {
            std::cerr << "Error: Gagal membuka file kalibrasi" << std::endl;
        }
        
    }
    return 0;
    
}