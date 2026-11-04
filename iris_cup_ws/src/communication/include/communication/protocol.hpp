// Protokol paket UDP IRIS CUP (8 byte, little-endian, CRC-8 poly 0x07).
//
// Perintah  (laptop -> ESP32):
//   [0]=0xAA [1]=seq [2..3]=steering int16 (0.01 deg) [4..5]=speed int16 (mm/s)
//   [6]=flags (bit0 enable, bit1 e-stop) [7]=CRC8([0..6])
// Telemetri (ESP32 -> laptop):
//   [0]=0x55 [1]=seq [2..3]=rpm int16 [4..5]=baterai uint16 (mV)
//   [6]=flags [7]=CRC8([0..6])
//
// File ini HARUS identik dengan implementasi di STM32/ESP32 (sepakati dengan electrical).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace iris_protocol {

constexpr uint8_t kCmdHeader = 0xAA;
constexpr uint8_t kTelHeader = 0x55;
constexpr size_t kPacketSize = 8;

using Packet = std::array<uint8_t, kPacketSize>;

enum CmdFlags : uint8_t { kFlagEnable = 0x01, kFlagEstop = 0x02 };

struct Command {
  uint8_t seq{0};
  int16_t steering_cdeg{0};  // 0.01 derajat
  int16_t speed_mm_s{0};
  uint8_t flags{0};
};

struct Telemetry {
  uint8_t seq{0};
  int16_t rpm{0};
  uint16_t battery_mv{0};
  uint8_t flags{0};
};

inline uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                         : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

inline void put_u16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}
inline uint16_t get_u16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}
inline void put_i16(uint8_t* p, int16_t v) { put_u16(p, static_cast<uint16_t>(v)); }
inline int16_t get_i16(const uint8_t* p) { return static_cast<int16_t>(get_u16(p)); }

inline Packet encode_command(const Command& c) {
  Packet p{};
  p[0] = kCmdHeader;
  p[1] = c.seq;
  put_i16(&p[2], c.steering_cdeg);
  put_i16(&p[4], c.speed_mm_s);
  p[6] = c.flags;
  p[7] = crc8(p.data(), 7);
  return p;
}

inline bool decode_command(const uint8_t* d, size_t len, Command& c) {
  if (len != kPacketSize || d[0] != kCmdHeader || crc8(d, 7) != d[7]) return false;
  c.seq = d[1];
  c.steering_cdeg = get_i16(&d[2]);
  c.speed_mm_s = get_i16(&d[4]);
  c.flags = d[6];
  return true;
}

inline Packet encode_telemetry(const Telemetry& t) {
  Packet p{};
  p[0] = kTelHeader;
  p[1] = t.seq;
  put_i16(&p[2], t.rpm);
  put_u16(&p[4], t.battery_mv);
  p[6] = t.flags;
  p[7] = crc8(p.data(), 7);
  return p;
}

inline bool decode_telemetry(const uint8_t* d, size_t len, Telemetry& t) {
  if (len != kPacketSize || d[0] != kTelHeader || crc8(d, 7) != d[7]) return false;
  t.seq = d[1];
  t.rpm = get_i16(&d[2]);
  t.battery_mv = get_u16(&d[4]);
  t.flags = d[6];
  return true;
}

}  // namespace iris_protocol
