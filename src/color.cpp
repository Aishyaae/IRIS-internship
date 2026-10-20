#include <opencv2/opencv.hpp>
#include <iostream>

// Deklarasi variabel global agar bisa diakses oleh fungsi callback
cv::Mat img, hls_img, mask;
int h_min = 0, h_max = 179;
int l_min = 0, l_max = 255;
int s_min = 0, s_max = 255;

// Fungsi callback yang akan dipanggil setiap kali trackbar digeser
void on_trackbar(int, void*) {
    // Buat scalar berdasarkan nilai trackbar saat ini
    cv::Scalar lower_bound(h_min, l_min, s_min);
    cv::Scalar upper_bound(h_max, l_max, s_max);

    // Lakukan thresholding / segmentasi warna pada gambar HLS
    cv::inRange(hls_img, lower_bound, upper_bound, mask);

    // Tampilkan hasil mask terbaru
    cv::imshow("Mask Output", mask);
}

int main() {
    // 1. Load foto Anda (ganti "path/ke/foto_anda.jpg" dengan lokasi foto asli)
    std::string image_path = "../docs/Object.jpeg"; 
    img = cv::imread(image_path);

    if (img.empty()) {
        std::cerr << "Error: Tidak dapat membaca gambar di " << image_path << std::endl;
        std::cerr << "Pastikan nama file dan jalurnya sudah benar!" << std::endl;
        return -1;
    }

    // 2. Konversi warna foto dari BGR ke HLS sekali saja di awal
    cv::cvtColor(img, hls_img, cv::COLOR_BGR2HLS);

    // 3. Buat jendela tampilan
    cv::namedWindow("Original Image", cv::WINDOW_NORMAL);
    cv::namedWindow("Trackbars", cv::WINDOW_NORMAL);
    cv::namedWindow("Mask Output", cv::WINDOW_NORMAL);

    // 4. Membuat 6 Trackbar dan hubungkan ke fungsi on_trackbar
    cv::createTrackbar("H Min", "Trackbars", &h_min, 179, on_trackbar);
    cv::createTrackbar("H Max", "Trackbars", &h_max, 179, on_trackbar);
    cv::createTrackbar("L Min", "Trackbars", &l_min, 255, on_trackbar);
    cv::createTrackbar("L Max", "Trackbars", &l_max, 255, on_trackbar);
    cv::createTrackbar("S Min", "Trackbars", &s_min, 255, on_trackbar);
    cv::createTrackbar("S Max", "Trackbars", &s_max, 255, on_trackbar);

    // Tampilkan gambar asli dan mask awal (kondisi default)
    cv::imshow("Original Image", img);
    on_trackbar(0, 0); 

    std::cout << "Program berjalan menggunakan foto. Geser trackbar pada jendela 'Trackbars'." << std::endl;
    std::cout << "Tekan 'q' atau 'ESC' pada jendela gambar untuk keluar dan melihat hasil angka." << std::endl;

    // Menunggu tombol ditekan (program akan terus responsif terhadap trackbar selama window aktif)
    while (true) {
        char key = (char)cv::waitKey(0); // waitKey(0) menahan program tanpa memakan CPU
        if (key == 'q' || key == 27) {
            // Cetak nilai akhir ke terminal sebelum program ditutup
            std::cout << "\n--- Nilai HLS Akhir Dicatat ---" << std::endl;
            std::cout << "H Min: " << h_min << ", H Max: " << h_max << std::endl;
            std::cout << "L Min: " << l_min << ", L Max: " << l_max << std::endl;
            std::cout << "S Min: " << s_min << ", S Max: " << s_max << std::endl;
            break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}
