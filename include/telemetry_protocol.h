#ifndef TELEMETRY_PROTOCOL_H
#define TELEMETRY_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

/**
 * @file telemetry_protocol.h
 * @brief Definitive 65-byte Little-Endian LoRa binary telemetry protocol.
 *
 * Matches config/frame.yaml and main.py in estacion-terrena-pc:
 *   - Header  (7 bytes) : sync (0xAA), frame_number (uint16), timestamp (uint32)
 *   - Payload (56 bytes): flight status, BMP280, 6-DoF IMU, GPS, pyro recovery pins
 *   - Trailer (2 bytes) : CRC-16/CCITT-FALSE over bytes 0..62
 */

constexpr uint8_t TELEMETRY_SYNC_BYTE        = 0xAA;
constexpr size_t  TELEMETRY_FRAME_SIZE       = 65;
constexpr size_t  TELEMETRY_CRC_PAYLOAD_SIZE = 63;

/**
 * @brief Sounding rocket flight stage codes (offset 7).
 */
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

/**
 * @brief Packed 65-byte Little-Endian binary telemetry frame.
 */
struct __attribute__((packed)) LoRaPacket {
    // Header (7 bytes, offsets 0..6)
    uint8_t  sync;           // Offset 0: 0xAA fixed sync byte
    uint16_t frame_number;   // Offset 1..2: Rolling packet sequence counter
    uint32_t timestamp;      // Offset 3..6: Mission elapsed time (ms)

    // Payload (56 bytes, offsets 7..62)
    uint8_t  status;         // Offset 7: Flight phase code
    float    pressure;       // Offset 8..11: Barometric pressure (hPa)
    float    altura_actual;  // Offset 12..15: Relative barometric altitude (m)
    float    accel_x;        // Offset 16..19: Linear acceleration X (g)
    float    accel_y;        // Offset 20..23: Linear acceleration Y (g)
    float    accel_z;        // Offset 24..27: Linear acceleration Z (g)
    float    gyro_x;         // Offset 28..31: Angular velocity X (deg/s)
    float    gyro_y;         // Offset 32..35: Angular velocity Y (deg/s)
    float    gyro_z;         // Offset 36..39: Angular velocity Z (deg/s)
    float    temp_imu;       // Offset 40..43: IMU internal temperature (deg C)
    float    latitude;       // Offset 44..47: GPS latitude (decimal degrees)
    float    longitude;      // Offset 48..51: GPS longitude (decimal degrees)
    float    altitud_gps;    // Offset 52..55: GPS altitude ASL (m)
    uint8_t  satellites;     // Offset 56: Tracked GPS satellites
    float    ground_speed;   // Offset 57..60: GPS ground speed (m/s)
    uint8_t  pin_frenado;    // Offset 61: Drogue pyro state (0 = OFF, 1 = ACTIVE)
    uint8_t  pin_principal;  // Offset 62: Main pyro state (0 = OFF, 1 = ACTIVE)

    // Trailer (2 bytes, offsets 63..64)
    uint16_t crc;            // Offset 63..64: CRC-16/CCITT-FALSE over bytes 0..62
};

static_assert(sizeof(LoRaPacket) == TELEMETRY_FRAME_SIZE,
              "LoRaPacket must be exactly 65 bytes");

/**
 * @brief Computes CRC-16/CCITT-FALSE over a byte buffer.
 *
 * Parameters: Polynomial 0x1021, Initial 0xFFFF, RefIn=false, RefOut=false, XorOut=0x0000.
 */
inline uint16_t calculateCrc16CcittFalse(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000U) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}

/**
 * @brief Validates the sync byte and CRC-16/CCITT-FALSE of a received LoRaPacket.
 */
inline bool verifyPacketCrc(const LoRaPacket &pkt) {
    if (pkt.sync != TELEMETRY_SYNC_BYTE) {
        return false;
    }
    const uint16_t computed = calculateCrc16CcittFalse(
        reinterpret_cast<const uint8_t *>(&pkt),
        TELEMETRY_CRC_PAYLOAD_SIZE
    );
    return computed == pkt.crc;
}

/**
 * @brief Maps a raw flight status code to its human-readable mission stage label.
 */
inline const char *statusToLabel(uint8_t status) {
    switch (status) {
        case 0x00:
            return "STANDBY";
        case 0x01:
            return "POWERED";
        case 0x02:
        case 0x03:
            return "BURNOUT";
        case 0x04:
        case 0x07:
            return "APOGEE";
        case 0x08:
        case 0x0F:
            return "DROGUE_ORD";
        case 0x10:
        case 0x1F:
            return "DROGUE_CONF";
        case 0x20:
        case 0x3F:
            return "MAIN_ORD";
        case 0x40:
        case 0x7F:
            return "MAIN_CONF";
        case 0x80:
        case 0xFF:
            return "LANDED";
        default:
            return "UNKNOWN";
    }
}

#endif // TELEMETRY_PROTOCOL_H
