#ifndef FLIGHT_SIMULATOR_H
#define FLIGHT_SIMULATOR_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#ifndef LORA_PACKET_DEFINED
#include "telemetry_protocol.h"
#endif

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// ============================================================================
//  290-SECOND SOUNDING ROCKET FLIGHT PROFILE SIMULATOR
// ============================================================================
//  Generates realistic sensor readings (barometer, 6-DoF IMU, GPS, pyro pins)
//  across 10 flight phases for bench testing before real rocket sensors are wired.
// ============================================================================

const unsigned long SIM_CYCLE_DURATION_MS = 290000UL;
const float         SIM_APOGEE_ALT_M      = 1850.0f;
const float         SIM_BASE_LATITUDE     = -34.603722f;
const float         SIM_BASE_LONGITUDE    = -58.381592f;
const float         SIM_BASE_ALT_MSL_M    = 25.0f;

// Linear interpolation between a and b
inline float simLerp(float a, float b, float t) {
  return a + t * (b - a);
}

// Smooth S-curve transition between edge0 and edge1
inline float simSmoothstep(float edge0, float edge1, float x) {
  float t = (x - edge0) / (edge1 - edge0);
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return t * t * (3.0f - 2.0f * t);
}

// Uniform random float in [minVal, maxVal]
inline float simRandomFloat(float minVal, float maxVal) {
  return minVal + ((float)rand() / (float)RAND_MAX) * (maxVal - minVal);
}

// Converts relative altitude (m) to barometric pressure (hPa)
inline float altitudeToPressureHpa(float altMeters) {
  const float P0 = 1013.25f;
  return P0 * powf(1.0f - (2.25577e-5f * altMeters), 5.25588f);
}

// Populates the 56-byte sensor payload inside `pkt` for mission time `t` (seconds)
inline void fillSimulatedSensorData(
    LoRaPacket& pkt,
    float       t,
    bool        enableFaults,
    bool&       shouldCorruptCrc
) {
  shouldCorruptCrc = false;

  // Safe baseline defaults
  float altitude      = 0.0f;
  float downrange     = 0.0f;
  float groundSpeed   = 0.0f;
  uint8_t sats        = 12;

  pkt.status        = 0x00; // STANDBY
  pkt.accel_x       = 0.0f;
  pkt.accel_y       = 0.0f;
  pkt.accel_z       = 1.0f;
  pkt.gyro_x        = 0.0f;
  pkt.gyro_y        = 0.0f;
  pkt.gyro_z        = 0.0f;
  pkt.temp_imu      = 28.5f;
  pkt.pin_frenado   = 0;
  pkt.pin_principal = 0;

  // --- Phase 1 (0 - 120 s): Pre-launch pad sway & fluctuating GPS sats ---
  if (t < 120.0f) {
    pkt.status   = 0x00; // STANDBY
    altitude     = simRandomFloat(-0.8f, 1.2f);
    float sway   = t * 0.4f;
    pkt.accel_x  = 0.06f * sinf(sway) + simRandomFloat(-0.03f, 0.03f);
    pkt.accel_y  = 0.05f * cosf(sway * 0.8f) + simRandomFloat(-0.03f, 0.03f);
    pkt.accel_z  = 1.00f + simRandomFloat(-0.02f, 0.02f);
    pkt.gyro_x   = 0.15f * sinf(t);
    pkt.gyro_y   = 0.12f * cosf(t * 0.9f);
    pkt.gyro_z   = 0.05f * sinf(t * 0.5f);
    pkt.temp_imu = 28.0f + 0.5f * sinf(t * 0.05f);
    sats         = (uint8_t)(8 + (int)(2.0f * sinf(t * 0.1f)));
  }
  // --- Phase 2 (120 - 155 s): Calibrated pre-launch baseline ---
  else if (t < 155.0f) {
    pkt.status = 0x00; // STANDBY
    altitude   = 0.0f;
  }
  // --- Phase 3 (155 - 161 s): Powered boost (9.8 g peak) ---
  else if (t < 161.0f) {
    pkt.status        = 0x01; // POWERED
    float burnTime    = t - 155.0f;
    float progress    = burnTime / 6.0f;
    float thrustCurve = sinf(progress * PI);

    pkt.accel_z  = 1.0f + (8.8f * thrustCurve);
    pkt.temp_imu = 28.5f + (burnTime * 0.8f);

    if (t >= 157.0f && t <= 158.5f) {
      // Simulated lateral wind gust / roll event
      pkt.accel_x = 9.70f;
      pkt.accel_y = -9.60f;
      pkt.gyro_x  = 45.0f;
      pkt.gyro_y  = -40.0f;
    } else {
      pkt.accel_x = 0.25f * sinf(burnTime * 4.0f);
      pkt.accel_y = 0.20f * cosf(burnTime * 4.0f);
      pkt.gyro_x  = 2.5f * sinf(burnTime * 2.0f);
      pkt.gyro_y  = 2.0f * cosf(burnTime * 2.0f);
    }

    pkt.gyro_z  = 120.0f * progress;
    altitude    = 480.0f * powf(progress, 2.2f);
    groundSpeed = 15.0f * progress;
    downrange   = 0.08f * progress;
  }
  // --- Phase 4 (161 - 180 s): Unpowered ballistic coast to apogee ---
  else if (t < 180.0f) {
    pkt.status     = 0x03; // BURNOUT
    float cTime    = t - 161.0f;
    float progress = cTime / 19.0f;
    float smooth   = simSmoothstep(0.0f, 1.0f, progress);

    pkt.accel_z  = simLerp(-1.05f, -0.98f, progress);
    pkt.accel_x  = 0.08f * sinf(cTime * 1.5f);
    pkt.accel_y  = 0.06f * cosf(cTime * 1.5f);
    pkt.gyro_x   = 1.2f * sinf(cTime);
    pkt.gyro_y   = 1.0f * cosf(cTime);
    pkt.gyro_z   = simLerp(120.0f, 15.0f, progress);
    pkt.temp_imu = simLerp(33.3f, 31.0f, progress);
    altitude     = simLerp(480.0f, SIM_APOGEE_ALT_M, smooth);
    groundSpeed  = simLerp(15.0f, 8.0f, progress);
    downrange    = simLerp(0.08f, 0.45f, smooth);
  }
  // --- Phase 5 (180.0 - 180.4 s): Apogee reached ---
  else if (t < 180.4f) {
    pkt.status   = 0x07; // APOGEE
    altitude     = SIM_APOGEE_ALT_M;
    pkt.accel_z  = 0.00f;
    pkt.accel_x  = 0.02f;
    pkt.accel_y  = 0.02f;
    pkt.gyro_x   = 0.5f;
    pkt.gyro_y   = 0.5f;
    pkt.gyro_z   = 5.0f;
    pkt.temp_imu = 30.5f;
    groundSpeed  = 7.5f;
    downrange    = 0.46f;
  }
  // --- Phase 6 (180.4 - 180.8 s): Drogue chute ejection order ---
  else if (t < 180.8f) {
    pkt.status      = 0x0F; // DROGUE_ORD
    pkt.pin_frenado = 1;
    altitude        = SIM_APOGEE_ALT_M - 2.0f;
    pkt.accel_z     = -0.85f;
    pkt.accel_x     = 0.40f;
    pkt.accel_y     = 0.35f;
    pkt.gyro_x      = 15.0f;
    pkt.gyro_y      = -12.0f;
    pkt.gyro_z      = 20.0f;
    pkt.temp_imu    = 30.5f;
    groundSpeed     = 8.0f;
    downrange       = 0.47f;
  }
  // --- Phase 7 (180.8 - 210.0 s): Drogue chute descent ---
  else if (t < 210.0f) {
    pkt.status      = 0x1F; // DROGUE_CONF
    pkt.pin_frenado = 1;
    float dTime     = t - 180.8f;
    float progress  = dTime / 29.2f;

    altitude     = simLerp(SIM_APOGEE_ALT_M - 2.0f, 450.0f, progress);
    pkt.accel_z  = 1.0f + 0.15f * sinf(dTime * 3.0f);
    pkt.accel_x  = 0.12f * sinf(dTime * 2.5f);
    pkt.accel_y  = 0.12f * cosf(dTime * 2.5f);
    pkt.gyro_x   = 4.0f * sinf(dTime * 1.5f);
    pkt.gyro_y   = 3.5f * cosf(dTime * 1.5f);
    pkt.gyro_z   = 8.0f * sinf(dTime * 0.5f);
    pkt.temp_imu = simLerp(30.5f, 29.2f, progress);
    groundSpeed  = 12.0f;
    downrange    = simLerp(0.47f, 0.75f, progress);

    if (enableFaults && t >= 184.0f && t <= 184.6f) {
      shouldCorruptCrc = true;
    }
  }
  // --- Phase 8 (210.0 - 210.4 s): Main chute ejection order ---
  else if (t < 210.4f) {
    pkt.status        = 0x3F; // MAIN_ORD
    pkt.pin_frenado   = 1;
    pkt.pin_principal = 1;
    altitude          = 450.0f;
    pkt.accel_z       = 3.2f;
    pkt.accel_x       = 0.60f;
    pkt.accel_y       = 0.55f;
    pkt.gyro_x        = 18.0f;
    pkt.gyro_y        = 16.0f;
    pkt.gyro_z        = -10.0f;
    pkt.temp_imu      = 29.0f;
    groundSpeed       = 10.0f;
    downrange         = 0.76f;
  }
  // --- Phase 9 (210.4 - 230.0 s): Main chute gentle descent ---
  else if (t < 230.0f) {
    pkt.status        = 0x7F; // MAIN_CONF
    pkt.pin_frenado   = 1;
    pkt.pin_principal = 1;
    float p           = (t - 210.4f) / 19.6f;
    float smoothP     = simSmoothstep(0.0f, 1.0f, p);

    altitude     = simLerp(450.0f, 0.0f, smoothP);
    pkt.accel_z  = 1.0f + 0.06f * sinf(p * 8.0f);
    pkt.accel_x  = 0.04f * sinf(p * 6.0f);
    pkt.accel_y  = 0.04f * cosf(p * 6.0f);
    pkt.gyro_x   = 1.5f * sinf(p * 5.0f);
    pkt.gyro_y   = 1.2f * cosf(p * 5.0f);
    pkt.gyro_z   = 0.5f;
    pkt.temp_imu = simLerp(29.0f, 28.5f, p);
    groundSpeed  = simLerp(10.0f, 3.2f, p);
    downrange    = simLerp(0.78f, 1.0f, p);
  }
  // --- Phase 10 (230 - 290 s): Landed on ground ---
  else {
    pkt.status        = 0xFF; // LANDED
    pkt.pin_frenado   = 1;
    pkt.pin_principal = 1;
    altitude          = 0.0f;
    downrange         = 1.0f;
  }

  // Fill barometer and GPS fields from computed state
  pkt.altura_actual = altitude;
  pkt.pressure      = altitudeToPressureHpa(altitude);
  pkt.latitude      = SIM_BASE_LATITUDE - (0.0032f * downrange);
  pkt.longitude     = SIM_BASE_LONGITUDE + (0.0055f * downrange);
  pkt.altitud_gps   = altitude + SIM_BASE_ALT_MSL_M;
  pkt.satellites    = sats;
  pkt.ground_speed  = groundSpeed;
}

// Helper class used by host C++ verification tests (and optional full simulation)
class FlightSimulator {
public:
  explicit FlightSimulator(uint32_t startMs = 0, bool enableFaults = true)
    : missionStartMs_(startMs),
      frameNumber_(0),
      tickCount_(0),
      crcErrorCount_(0),
      enableFaults_(enableFaults) {}

  void reset(uint32_t startMs) {
    missionStartMs_ = startMs;
    frameNumber_    = 0;
    tickCount_      = 0;
    crcErrorCount_  = 0;
  }

  // Returns true if a 65-byte packet should be transmitted at nowMs
  bool buildNextPacket(uint32_t nowMs, LoRaPacket& outPacket) {
    uint32_t elapsedMs = nowMs - missionStartMs_;

    // Wrap 290-second simulation cleanly BEFORE stamping packet timestamp
    if (elapsedMs >= SIM_CYCLE_DURATION_MS) {
      missionStartMs_ = nowMs;
      elapsedMs       = 0;
      crcErrorCount_  = 0;
    }

    float t = (float)elapsedMs / 1000.0f;
    uint32_t currentTick = tickCount_++;

    // Optional packet-loss test window (165.0s - 168.0s): transmits 1 out of 4 ticks
    if (enableFaults_ && t >= 165.0f && t <= 168.0f) {
      if ((currentTick % 4U) != 0U) {
        frameNumber_++;
        return false;
      }
    }

    outPacket.sync         = 0xAA;
    outPacket.frame_number = frameNumber_;
    outPacket.timestamp    = elapsedMs;

    bool corruptCrc = false;
    fillSimulatedSensorData(outPacket, t, enableFaults_, corruptCrc);

    // CRC-16/CCITT-FALSE over bytes 0..62
    uint16_t crc = 0xFFFF;
    const uint8_t* raw = (const uint8_t*)&outPacket;
    for (size_t i = 0; i < sizeof(LoRaPacket) - 2; i++) {
      crc ^= ((uint16_t)raw[i] << 8);
      for (int bit = 0; bit < 8; bit++) {
        crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
      }
    }

    if (corruptCrc && crcErrorCount_ < 3) {
      crc ^= 0xAAAA;
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
  bool     enableFaults_;
};

#endif // FLIGHT_SIMULATOR_H
