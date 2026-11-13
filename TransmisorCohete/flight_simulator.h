#ifndef FLIGHT_SIMULATOR_H
#define FLIGHT_SIMULATOR_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "telemetry_protocol.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

/**
 * @file flight_simulator.h
 * @brief 290-second sounding rocket flight profile simulator (10 phases, 10 Hz).
 *
 * Generates realistic barometric, inertial, GPS, and pyrotechnic telemetry
 * matching estacion-terrena-pc's test suite and dashboard alerts, with fixes for
 * odd-parity packet-loss lockouts, cycle wrap-around synchronization, and
 * 32-bit floating-point trigonometry on the ESP32-C3 RISC-V core.
 */

namespace FlightSimConfig {
constexpr uint32_t TOTAL_CYCLE_MS  = 290000UL;
constexpr float    APOGEE_ALT_M    = 1850.0f;
constexpr float    BASE_LATITUDE   = -34.603722f;
constexpr float    BASE_LONGITUDE  = -58.381592f;
constexpr float    BASE_ALT_MSL_M  = 25.0f;
} // namespace FlightSimConfig

struct FlightState {
    float   altitude;
    float   velocity;
    float   accelX_g;
    float   accelY_g;
    float   accelZ_g;
    float   gyroX_dps;
    float   gyroY_dps;
    float   gyroZ_dps;
    float   tempImu_c;
    float   pressure_hpa;
    float   groundSpeed_ms;
    uint8_t statusCode;
    uint8_t satellites;
    uint8_t pinFrenado;
    uint8_t pinPrincipal;
    float   downrangeDrift;
    bool    corruptCrc;
};

class FlightSimulator {
public:
    explicit FlightSimulator(uint32_t startMs = 0)
        : missionStartMs_(startMs),
          frameNumber_(0),
          tickCount_(0),
          crcErrorCount_(0) {}

    void reset(uint32_t startMs) {
        missionStartMs_ = startMs;
        frameNumber_    = 0;
        tickCount_      = 0;
        crcErrorCount_  = 0;
    }

    uint16_t getFrameNumber() const { return frameNumber_; }
    uint8_t  getCrcErrorCount() const { return crcErrorCount_; }

    static float lerp(float a, float b, float t) {
        return a + t * (b - a);
    }

    static float smoothstep(float edge0, float edge1, float x) {
        float t = (x - edge0) / (edge1 - edge0);
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        return t * t * (3.0f - 2.0f * t);
    }

    static float randomFloat(float minVal, float maxVal) {
        return minVal + (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) *
                            (maxVal - minVal);
    }

    static float altitudeToPressure(float altMeters) {
        constexpr float P0 = 1013.25f;
        return P0 * powf(1.0f - (2.25577e-5f * altMeters), 5.25588f);
    }

    static FlightState calculateFlight(float t) {
        FlightState s{};
        s.accelZ_g       = 1.000f;
        s.tempImu_c      = 28.5f;
        s.statusCode     = STATUS_STANDBY;
        s.satellites     = 12;

        // Phase 1 (0 - 120 s): Pre-launch pad sway & fluctuating sats
        if (t < 120.0f) {
            s.statusCode = STATUS_STANDBY;
            s.altitude   = randomFloat(-0.8f, 1.2f);
            float sway   = t * 0.4f;
            s.accelX_g   = 0.06f * sinf(sway) + randomFloat(-0.03f, 0.03f);
            s.accelY_g   = 0.05f * cosf(sway * 0.8f) + randomFloat(-0.03f, 0.03f);
            s.accelZ_g   = 1.000f + randomFloat(-0.02f, 0.02f);
            s.gyroX_dps  = 0.15f * sinf(t);
            s.gyroY_dps  = 0.12f * cosf(t * 0.9f);
            s.gyroZ_dps  = 0.05f * sinf(t * 0.5f);
            s.tempImu_c  = 28.0f + 0.5f * sinf(t * 0.05f);
            s.satellites = static_cast<uint8_t>(8 + static_cast<int>(2.0f * sinf(t * 0.1f)));
            s.pressure_hpa = altitudeToPressure(s.altitude);
            return s;
        }

        // Phase 2 (120 - 155 s): Pre-launch calibrated baseline
        if (t < 155.0f) {
            s.statusCode   = STATUS_STANDBY;
            s.accelZ_g     = 1.000f;
            s.pressure_hpa = altitudeToPressure(0.0f);
            return s;
        }

        // Phase 3 (155 - 161 s): Powered boost (9.8g peak, lateral G alert at 157.0-158.5s)
        if (t < 161.0f) {
            s.statusCode = STATUS_POWERED;
            float burnTime     = t - 155.0f;
            float burnProgress = burnTime / 6.0f;
            float thrustCurve  = sinf(burnProgress * PI);

            s.accelZ_g  = 1.0f + (8.8f * thrustCurve);
            s.tempImu_c = 28.5f + (burnTime * 0.8f);

            if (t >= 157.0f && t <= 158.5f) {
                s.accelX_g  = 9.70f;
                s.accelY_g  = -9.60f;
                s.gyroX_dps = 45.0f;
                s.gyroY_dps = -40.0f;
            } else {
                s.accelX_g  = 0.25f * sinf(burnTime * 4.0f);
                s.accelY_g  = 0.20f * cosf(burnTime * 4.0f);
                s.gyroX_dps = 2.5f * sinf(burnTime * 2.0f);
                s.gyroY_dps = 2.0f * cosf(burnTime * 2.0f);
            }

            s.gyroZ_dps      = 120.0f * burnProgress;
            s.velocity       = 195.0f * powf(burnProgress, 1.4f);
            s.altitude       = 480.0f * powf(burnProgress, 2.2f);
            s.groundSpeed_ms = 15.0f * burnProgress;
            s.downrangeDrift = 0.08f * burnProgress;
            s.pressure_hpa   = altitudeToPressure(s.altitude);
            return s;
        }

        // Phase 4 (161 - 180 s): Unpowered ballistic coast to apogee
        if (t < 180.0f) {
            s.statusCode = STATUS_BURNOUT;
            float coastTime     = t - 161.0f;
            float coastProgress = coastTime / 19.0f;
            float p             = smoothstep(0.0f, 1.0f, coastProgress);

            s.accelZ_g       = lerp(-1.05f, -0.98f, coastProgress);
            s.accelX_g       = 0.08f * sinf(coastTime * 1.5f);
            s.accelY_g       = 0.06f * cosf(coastTime * 1.5f);
            s.gyroX_dps      = 1.2f * sinf(coastTime);
            s.gyroY_dps      = 1.0f * cosf(coastTime);
            s.gyroZ_dps      = lerp(120.0f, 15.0f, coastProgress);
            s.tempImu_c      = lerp(33.3f, 31.0f, coastProgress);
            s.velocity       = lerp(195.0f, 0.0f, coastProgress);
            s.altitude       = lerp(480.0f, FlightSimConfig::APOGEE_ALT_M, p);
            s.groundSpeed_ms = lerp(15.0f, 8.0f, coastProgress);
            s.downrangeDrift = lerp(0.08f, 0.45f, p);
            s.pressure_hpa   = altitudeToPressure(s.altitude);
            return s;
        }

        // Phase 5 (180.0 - 180.4 s): Apogee
        if (t < 180.4f) {
            s.statusCode     = STATUS_APOGEE;
            s.altitude       = FlightSimConfig::APOGEE_ALT_M;
            s.accelZ_g       = 0.00f;
            s.accelX_g       = 0.02f;
            s.accelY_g       = 0.02f;
            s.gyroX_dps      = 0.5f;
            s.gyroY_dps      = 0.5f;
            s.gyroZ_dps      = 5.0f;
            s.tempImu_c      = 30.5f;
            s.groundSpeed_ms = 7.5f;
            s.downrangeDrift = 0.46f;
            s.pressure_hpa   = altitudeToPressure(s.altitude);
            return s;
        }

        // Phase 6 (180.4 - 180.8 s): Drogue ejection command
        if (t < 180.8f) {
            s.statusCode     = STATUS_DROGUE_ORD;
            s.altitude       = FlightSimConfig::APOGEE_ALT_M - 2.0f;
            s.velocity       = -4.0f;
            s.accelZ_g       = -0.85f;
            s.accelX_g       = 0.40f;
            s.accelY_g       = 0.35f;
            s.gyroX_dps      = 15.0f;
            s.gyroY_dps      = -12.0f;
            s.gyroZ_dps      = 20.0f;
            s.tempImu_c      = 30.5f;
            s.pinFrenado     = 1;
            s.groundSpeed_ms = 8.0f;
            s.downrangeDrift = 0.47f;
            s.pressure_hpa   = altitudeToPressure(s.altitude);
            return s;
        }

        // Phase 7 (180.8 - 210.0 s): Drogue descent (CRC fault injection 184.0-184.6s)
        if (t < 210.0f) {
            s.statusCode = STATUS_DROGUE_CONF;
            s.pinFrenado = 1;
            float dTime     = t - 180.8f;
            float dProgress = dTime / 29.2f;

            s.altitude       = lerp(FlightSimConfig::APOGEE_ALT_M - 2.0f, 450.0f, dProgress);
            s.velocity       = -48.0f;
            s.accelZ_g       = 1.0f + 0.15f * sinf(dTime * 3.0f);
            s.accelX_g       = 0.12f * sinf(dTime * 2.5f);
            s.accelY_g       = 0.12f * cosf(dTime * 2.5f);
            s.gyroX_dps      = 4.0f * sinf(dTime * 1.5f);
            s.gyroY_dps      = 3.5f * cosf(dTime * 1.5f);
            s.gyroZ_dps      = 8.0f * sinf(dTime * 0.5f);
            s.tempImu_c      = lerp(30.5f, 29.2f, dProgress);
            s.downrangeDrift = lerp(0.47f, 0.75f, dProgress);
            s.groundSpeed_ms = 12.0f;

            if (t >= 184.0f && t <= 184.6f) {
                s.corruptCrc = true;
            }

            s.pressure_hpa = altitudeToPressure(s.altitude);
            return s;
        }

        // Phase 8 (210.0 - 210.4 s): Main chute ejection command
        if (t < 210.4f) {
            s.statusCode     = STATUS_MAIN_ORD;
            s.pinFrenado     = 1;
            s.pinPrincipal   = 1;
            s.altitude       = 450.0f;
            s.velocity       = -45.0f;
            s.accelZ_g       = 3.2f;
            s.accelX_g       = 0.60f;
            s.accelY_g       = 0.55f;
            s.gyroX_dps      = 18.0f;
            s.gyroY_dps      = 16.0f;
            s.gyroZ_dps      = -10.0f;
            s.tempImu_c      = 29.0f;
            s.groundSpeed_ms = 10.0f;
            s.downrangeDrift = 0.76f;
            s.pressure_hpa   = altitudeToPressure(s.altitude);
            return s;
        }

        // Phase 9 (210.4 - 230.0 s): Main chute gentle descent
        if (t < 230.0f) {
            s.statusCode   = STATUS_MAIN_CONF;
            s.pinFrenado   = 1;
            s.pinPrincipal = 1;
            float p        = (t - 210.4f) / 19.6f;
            float smoothP  = smoothstep(0.0f, 1.0f, p);

            s.altitude       = lerp(450.0f, 0.0f, smoothP);
            s.velocity       = lerp(-12.0f, -4.5f, p);
            s.accelZ_g       = 1.0f + 0.06f * sinf(p * 8.0f);
            s.accelX_g       = 0.04f * sinf(p * 6.0f);
            s.accelY_g       = 0.04f * cosf(p * 6.0f);
            s.gyroX_dps      = 1.5f * sinf(p * 5.0f);
            s.gyroY_dps      = 1.2f * cosf(p * 5.0f);
            s.gyroZ_dps      = 0.5f;
            s.tempImu_c      = lerp(29.0f, 28.5f, p);
            s.groundSpeed_ms = lerp(10.0f, 3.2f, p);
            s.downrangeDrift = lerp(0.78f, 1.0f, p);
            s.pressure_hpa   = altitudeToPressure(s.altitude);
            return s;
        }

        // Phase 10 (230 - 290 s): Landed on ground
        s.statusCode     = STATUS_LANDED;
        s.pinFrenado     = 1;
        s.pinPrincipal   = 1;
        s.accelZ_g       = 1.000f;
        s.pressure_hpa   = altitudeToPressure(0.0f);
        s.downrangeDrift = 1.0f;
        return s;
    }

    /**
     * @brief Builds the next 65-byte telemetry frame for the given timestamp.
     * @return True if the packet should be transmitted; false if intentionally dropped
     *         during the simulated packet-loss test window (165.0s - 168.0s).
     */
    bool buildNextPacket(uint32_t nowMs, LoRaPacket &outPacket) {
        uint32_t elapsedMs = nowMs - missionStartMs_;

        // Handle 290-second wrap-around BEFORE stamping packet timestamp
        if (elapsedMs >= FlightSimConfig::TOTAL_CYCLE_MS) {
            missionStartMs_ = nowMs;
            elapsedMs       = 0;
            crcErrorCount_  = 0;
        }

        const float t = static_cast<float>(elapsedMs) / 1000.0f;
        const FlightState flight = calculateFlight(t);

        // Separate tick counter prevents odd-parity modulo lockout during loss window
        const uint32_t currentTick = tickCount_++;
        if (t >= 165.0f && t <= 168.0f) {
            if ((currentTick % 4U) != 0U) {
                frameNumber_++;
                return false;
            }
        }

        // Header (7 bytes)
        outPacket.sync         = TELEMETRY_SYNC_BYTE;
        outPacket.frame_number = frameNumber_;
        outPacket.timestamp    = elapsedMs;

        // Payload (56 bytes)
        outPacket.status        = flight.statusCode;
        outPacket.pressure      = flight.pressure_hpa;
        outPacket.altura_actual = flight.altitude;
        outPacket.accel_x       = flight.accelX_g;
        outPacket.accel_y       = flight.accelY_g;
        outPacket.accel_z       = flight.accelZ_g;
        outPacket.gyro_x        = flight.gyroX_dps;
        outPacket.gyro_y        = flight.gyroY_dps;
        outPacket.gyro_z        = flight.gyroZ_dps;
        outPacket.temp_imu      = flight.tempImu_c;
        outPacket.latitude      = FlightSimConfig::BASE_LATITUDE - (0.0032f * flight.downrangeDrift);
        outPacket.longitude     = FlightSimConfig::BASE_LONGITUDE + (0.0055f * flight.downrangeDrift);
        outPacket.altitud_gps   = flight.altitude + FlightSimConfig::BASE_ALT_MSL_M;
        outPacket.satellites    = flight.satellites;
        outPacket.ground_speed  = flight.groundSpeed_ms;
        outPacket.pin_frenado   = flight.pinFrenado;
        outPacket.pin_principal = flight.pinPrincipal;

        // Trailer (2 bytes): CRC-16/CCITT-FALSE over bytes 0..62
        uint16_t crc = calculateCrc16CcittFalse(
            reinterpret_cast<const uint8_t *>(&outPacket),
            TELEMETRY_CRC_PAYLOAD_SIZE
        );

        if (flight.corruptCrc && crcErrorCount_ < 3) {
            crc ^= 0xAAAAU;
            crcErrorCount_++;
        }

        outPacket.crc = crc;
        frameNumber_++;
        return true;
    }

private:
    uint32_t missionStartMs_;
    uint16_t frameNumber_;
    uint32_t tickCount_;
    uint8_t  crcErrorCount_;
};

#endif // FLIGHT_SIMULATOR_H
