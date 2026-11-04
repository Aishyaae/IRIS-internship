#include <arpa/inet.h> // konversi alamat htons, inet_pton
#include <endian.h> // htole32
#include <netinet/in.h> // definisiin struktur alamat
#include <sys/socket.h> // fungsi socket() sendto()
#include <unistd.h> // close() nutup socket

#include <algorithm> // clamp, max
#include <chrono> // satuan waktu pada timer
#include <cstdint> // tipe integer ukuran pasti int32_t, uint8_t, uint32_t
#include <cstring> // memcpy
#include <memory> // make_shared untuk buat node
#include <stdexcept> // runtime_error kalo ada socket gagal dibuat
#include <string> 

#include "rclcpp/rclcpp.hpp" //buat node, publisher, subscriber, timer
#include "std_msgs/msg/int32_multi_array.hpp" // tipe pesan ros2 berupa array integer 32 bit untuk format perintah

// membuat kelas UdpSender yang mewarisi kelas dasar rclcpp::Node
class UdpSenderNode : public rclcpp::Node {
public:
  UdpSenderNode() : Node("udp_sender_node") {
    // constructor dipanggil saat objek dibuat. memberi nama node udp_sender_node


    // declare parameter ros. kalo gak diubah dari luar, dipakai nilai default
    host = declare_parameter<std::string>("host", "127.0.0.1"); // IP tujuan
    port = declare_parameter<int>("port", 12151); // port UDP simulasi
    rate_hz = declare_parameter<int>("rate_hz", 20); // seberapa sering paket dikirim per detik
    timeout_ms = declare_parameter<int>("cmd_timeout_ms", 500); // batas waktu tanpa perintah baru sebelum mobil dihentikan

    // koneksi socket udp
    setup_socket();

    // subcribe ke package master
    sub = create_subscription<std_msgs::msg::Int32MultiArray>(
        "/master/geometry_control", 10, // nama topik yg didengarkan, 10 ukuran antrean (kalo 10 pesan menumpuk, yang paling lama dibuang)
        std::bind(&UdpSenderNode::master_callback, this, std::placeholders::_1)); // hubungin topic ke cmd_callback, this menunjuk objek ini, _1 adalah tempat pesan masuk

    // timer pengiriman yang manggil timer_callback secara periodik
    timer = create_wall_timer( 
        std::chrono::milliseconds(1000 / std::max(1, rate_hz)), // ubah frekuensi jadi periode ms (20hz -> 50ms)
        // std::max cegah pembagian dengan 0 kalau rate 0
        std::bind(&UdpSenderNode::timer_callback, this));

    last_cmd_time = now(); // catat waktu sekarang sebagai "terakhir terima perintah". untuk hitungan timeout
    RCLCPP_INFO(get_logger(), "Mengirim UDP Langsung ke Gazebo di %s:%d pada %d Hz", host.c_str(), port, rate_hz);
  }

  // destructor dipanggil saat node dihancurkan (Ctrl+C / shutdown)
  ~UdpSenderNode() override {
    if (sock >= 0) { // hanya valid kalo socket jadi
      pwm = 0;  // paket stop, steering tetap
      send_packet(); // kirim paket terakhir
      RCLCPP_INFO(get_logger(), "Stopped"); // cetak log
      close(sock); // tutup socket
    }
  }

private:
  static constexpr char kHeader[4] = {'I', 'R', 'I', 'S'}; // static artinya satu salinan dipakai di semua objek
  // constextpr nilainya tetap dan sdh ditentukan saat kompilasi. simulasi hanya nerima paket yg diawali "iris"
  static constexpr size_t kPacketSize = 12; // ukuran paket

  // membuat socket UDP dan menyiapkan alamat tujuan
  void setup_socket() {
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    // hasilnya berupa file descriptor (angka). nilai negatif berarti gagal
    if (sock < 0) {
      throw std::runtime_error("Failed to create socket"); // kalo gagal, lempar exception pesan error
    }

    addr = {}; // kosongin struct alamat agar gaada sisa data acak
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) { // ubah ip teks jadi 4 byte biner di sin_addr dan return 1 kalo berhasil
      close(sock); // kalo ip gk valid, tutup socket
      sock = -1; // tandai
      throw std::runtime_error("Invalid host: " + host); // lempar error
    }
  }

  // isi buffer 12 byte: "IRIS" | pwm (int32 LE) | steer (int32 LE)
  void build_packet(uint8_t (&buf)[kPacketSize], int32_t pwm, int32_t steer) {
    // membatasi nilai rentang untuk simulasi
    pwm = std::clamp<int32_t>(pwm, -255, 255);
    steer = std::clamp<int32_t>(steer, -90, 90);
    // static_cast mengubah int bertanda jadi tak bertanda dengan pola bit yg sama, karena htole bekerja pada uint32_t
    // htole32 memastikan urutan byte little endian di mesin apapun
    uint32_t pwm_le = htole32(static_cast<uint32_t>(pwm));
    uint32_t steer_le = htole32(static_cast<uint32_t>(steer));

    // menyusun paket byte demi byte
    std::memcpy(buf, kHeader, sizeof(kHeader));
    std::memcpy(buf + 4, &pwm_le, sizeof(pwm_le));
    std::memcpy(buf + 8, &steer_le, sizeof(steer_le));
  }

  void send_packet() {
    if (sock < 0) return;
    uint8_t packet[kPacketSize];
    build_packet(packet, pwm, steer);
    
    // mengirim data (socket yg dipakai, data, panjangnya, ..., ukuran struct alamat, hasil negatif berarti gagal catat jadi error)
    if (sendto(sock, packet, sizeof(packet), 0, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "sendto failed");
    }
  }

  // simpan perintah terbaru dari topic
  // msg pointer ke pesan yg masuk
  void master_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 2) { // array harus ada min 2 elemen
      RCLCPP_WARN(get_logger(), "cmd_motor needs 2 values: [pwm, steer]"); // kalo kurang, beri pesan dan abaikan
      return;
    }
    // menyimpan perintah terbaru
    pwm = msg->data[0]; // index 0 pwm
    steer = msg->data[1]; // index 1 steer
    last_cmd_time = now(); // memperbarui waktu terakhir ada perintah
  }

  // dipanggil periodik oleh timer: kirim perintah terakhir ke simulasi
  void timer_callback() {
    // pengaman: tidak ada perintah baru -> berhenti
    if ((now() - last_cmd_time).nanoseconds() > timeout_ms * 1000000LL) {
      pwm = 0;
    }
    send_packet(); // kirim paket terakhir
  }

  // --- member ---
  int sock = -1;
  sockaddr_in addr{};
  std::string host;
  int port;
  int rate_hz;
  int timeout_ms;

  int32_t pwm = 0;
  int32_t steer = 0;
  rclcpp::Time last_cmd_time{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub;
  rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // buat node dan spin jalankan event loop menunggu pesan dan timer sampai ctrl+c
  // setelah spin selesai, node dihancurkan dan destructor memanggil paket stop
  rclcpp::spin(std::make_shared<UdpSenderNode>());
  rclcpp::shutdown(); // membersihkan resource ros, lalu program selesai
  return 0;
}