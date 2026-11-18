#ifndef TELEMETRY_PROTOCOL_H
#define TELEMETRY_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

// ============================================================================
//  65-BYTE LITTLE-ENDIAN LORA TELEMETRY PROTOCOL REFERENCE HEADER
// ============================================================================
//  Matches estacion-terrena-pc/config/frame.yaml and main.py:
//    - Header  (7 bytes, offsets 0..6)  : sync (0xAA), frame_number, timestamp
//    - Payload (56 bytes, offsets 7..62): status, BMP280, IMU, GPS, pyro pins
//    - Trailer (2 bytes, offsets 63..64): CRC-16/CCITT-FALSE over bytes 0..62
// ============================================================================

const uint8_t SYNC_BYTE        = 0xAA;
const size_t  PACKET_SIZE      = 65;
const size_t  CRC_PAYLOAD_SIZE = 63;

const uint8_t GS_SYNC_BYTE        = 0xAB;
const uint8_t GS_PROTO_VERSION    = 0x10;
const size_t  GS_HEADER_SIZE      = 32;
const size_t  USB_PACKET_SIZE     = 97;

enum FlightStatus : uint8_t {
  STATUS_STANDBY     = 0x00,
  STATUS_POWERED     = 0x01,
  STATUS_BURNOUT     = 0x03,
  STATUS_APOGEE      = 0x07,
  STATUS_DROGUE_ORD  = 0x0F,
  STATUS_DROGUE_CONF = 0x1F,
  STATUS_MAIN_ORD    = 0x3F,
  STATUS_MAIN_CONF   = 0x7F,
  STATUS_LANDED      = 0xFF
};

#ifndef LORA_PACKET_DEFINED
#define LORA_PACKET_DEFINED
struct __attribute__((packed)) LoRaPacket {
  // Header (7 bytes, offsets 0..6)
  uint8_t  sync;           // [0]      Fixed sync byte (0xAA)
  uint16_t frame_number;   // [1..2]   Sequential packet counter (0..65535)
  uint32_t timestamp;      // [3..6]   Mission elapsed time (ms)

  // Payload (56 bytes, offsets 7..62)
  uint8_t  status;         // [7]      Flight stage code
  float    pressure;       // [8..11]  Barometric pressure (hPa)
  float    altura_actual;  // [12..15] Barometric altitude (m)
  float    accel_x;        // [16..19] IMU Acceleration X (g)
  float    accel_y;        // [20..23] IMU Acceleration Y (g)
  float    accel_z;        // [24..27] IMU Acceleration Z (g)
  float    gyro_x;         // [28..31] IMU Angular velocity X (deg/s)
  float    gyro_y;         // [32..35] IMU Angular velocity Y (deg/s)
  float    gyro_z;         // [36..39] IMU Angular velocity Z (deg/s)
  float    temp_imu;       // [40..43] IMU Temperature (deg C)
  float    latitude;       // [44..47] GPS Latitude (decimal degrees)
  float    longitude;      // [48..51] GPS Longitude (decimal degrees)
  float    altitud_gps;    // [52..55] GPS Altitude ASL (m)
  uint8_t  satellites;     // [56]     Tracked GPS satellites
  float    ground_speed;   // [57..60] GPS Ground speed (m/s)
  uint8_t  pin_frenado;    // [61]     Drogue chute pyro state (0 = OFF, 1 = ACTIVE)
  uint8_t  pin_principal;  // [62]     Main chute pyro state (0 = OFF, 1 = ACTIVE)

  // Trailer (2 bytes, offsets 63..64)
  uint16_t crc;            // [63..64] CRC-16/CCITT-FALSE over bytes 0..62
};
#endif // LORA_PACKET_DEFINED

static_assert(sizeof(LoRaPacket) == 65, "LoRaPacket must be exactly 65 bytes");

struct __attribute__((packed)) GroundStationHeader {
  uint8_t  gs_sync;        // [0]      0xAB
  uint8_t  gs_flags;       // [1]      Bits 7..4 = proto v1; bit0=crc_ok, bit1=link_active, bit2=tx_reboot
  int16_t  rssi;           // [2..3]   RSSI (dBm)
  float    snr;            // [4..7]   SNR (dB)
  uint32_t gs_uptime_ms;   // [8..11]  GS millis()
  uint32_t rx_packets;     // [12..15] Valid packets RX
  uint32_t rx_lost;        // [16..19] Sequence loss count
  uint32_t rx_crc_errors;  // [20..23] CRC error count
  uint32_t rx_size_errors; // [24..27] Size/sync error count
  uint16_t tx_reboots;     // [28..29] TX reboot/cycle count
  uint16_t gs_crc;         // [30..31] CRC-16/CCITT-FALSE over bytes 0..29
};

struct __attribute__((packed)) UsbTelemetryPacket {
  GroundStationHeader gs;     // [0..31]
  LoRaPacket          rocket; // [32..96]
};

static_assert(sizeof(GroundStationHeader) == 32, "GroundStationHeader must be 32 bytes");
static_assert(sizeof(UsbTelemetryPacket) == 97, "UsbTelemetryPacket must be 97 bytes");

// Computes CRC-16/CCITT-FALSE (Poly 0x1021, Init 0xFFFF, RefIn=false, RefOut=false, XorOut=0x0000)
inline uint16_t calculateCRC16(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= ((uint16_t)data[i] << 8);
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc = (crc << 1);
      }
    }
  }
  return crc;
}

inline bool verifyPacketCrc(const LoRaPacket& pkt) {
  if (pkt.sync != SYNC_BYTE) {
    return false;
  }
  uint16_t computed = calculateCRC16((const uint8_t*)&pkt, CRC_PAYLOAD_SIZE);
  return computed == pkt.crc;
}

inline const char* getStatusName(uint8_t status) {
  switch (status) {
    case 0x00:             return "STANDBY";
    case 0x01:             return "POWERED";
    case 0x02: case 0x03:  return "BURNOUT";
    case 0x04: case 0x07:  return "APOGEE";
    case 0x08: case 0x0F:  return "DROGUE_ORD";
    case 0x10: case 0x1F:  return "DROGUE_CONF";
    case 0x20: case 0x3F:  return "MAIN_ORD";
    case 0x40: case 0x7F:  return "MAIN_CONF";
    case 0x80: case 0xFF:  return "LANDED";
    default:               return "UNKNOWN";
  }
}

#endif // TELEMETRY_PROTOCOL_H
