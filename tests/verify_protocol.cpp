#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>

#include "telemetry_protocol.h"
#include "lora_config.h"
#include "flight_simulator.h"

int main(int argc, char** argv) {
  // 1. Verify exact 65-byte size and field offsets matching config/frame.yaml
  static_assert(sizeof(LoRaPacket) == 65, "Frame size must be 65 bytes");
  static_assert(offsetof(LoRaPacket, sync) == 0, "sync offset mismatch");
  static_assert(offsetof(LoRaPacket, frame_number) == 1, "frame_number offset mismatch");
  static_assert(offsetof(LoRaPacket, timestamp) == 3, "timestamp offset mismatch");
  static_assert(offsetof(LoRaPacket, status) == 7, "status offset mismatch");
  static_assert(offsetof(LoRaPacket, pressure) == 8, "pressure offset mismatch");
  static_assert(offsetof(LoRaPacket, altura_actual) == 12, "altura_actual offset mismatch");
  static_assert(offsetof(LoRaPacket, accel_x) == 16, "accel_x offset mismatch");
  static_assert(offsetof(LoRaPacket, accel_y) == 20, "accel_y offset mismatch");
  static_assert(offsetof(LoRaPacket, accel_z) == 24, "accel_z offset mismatch");
  static_assert(offsetof(LoRaPacket, gyro_x) == 28, "gyro_x offset mismatch");
  static_assert(offsetof(LoRaPacket, gyro_y) == 32, "gyro_y offset mismatch");
  static_assert(offsetof(LoRaPacket, gyro_z) == 36, "gyro_z offset mismatch");
  static_assert(offsetof(LoRaPacket, temp_imu) == 40, "temp_imu offset mismatch");
  static_assert(offsetof(LoRaPacket, latitude) == 44, "latitude offset mismatch");
  static_assert(offsetof(LoRaPacket, longitude) == 48, "longitude offset mismatch");
  static_assert(offsetof(LoRaPacket, altitud_gps) == 52, "altitud_gps offset mismatch");
  static_assert(offsetof(LoRaPacket, satellites) == 56, "satellites offset mismatch");
  static_assert(offsetof(LoRaPacket, ground_speed) == 57, "ground_speed offset mismatch");
  static_assert(offsetof(LoRaPacket, pin_frenado) == 61, "pin_frenado offset mismatch");
  static_assert(offsetof(LoRaPacket, pin_principal) == 62, "pin_principal offset mismatch");
  static_assert(offsetof(LoRaPacket, crc) == 63, "crc offset mismatch");

  // Verify 32-byte GroundStationHeader + 65-byte LoRaPacket = 97-byte USB frame
  static_assert(sizeof(GroundStationHeader) == 32, "GroundStationHeader must be 32 bytes");
  static_assert(sizeof(UsbTelemetryPacket) == 97, "UsbTelemetryPacket must be 97 bytes");
  static_assert(offsetof(GroundStationHeader, gs_sync) == 0, "gs_sync offset mismatch");
  static_assert(offsetof(GroundStationHeader, gs_flags) == 1, "gs_flags offset mismatch");
  static_assert(offsetof(GroundStationHeader, rssi) == 2, "rssi offset mismatch");
  static_assert(offsetof(GroundStationHeader, snr) == 4, "snr offset mismatch");
  static_assert(offsetof(GroundStationHeader, gs_uptime_ms) == 8, "gs_uptime_ms offset mismatch");
  static_assert(offsetof(GroundStationHeader, rx_packets) == 12, "rx_packets offset mismatch");
  static_assert(offsetof(GroundStationHeader, rx_lost) == 16, "rx_lost offset mismatch");
  static_assert(offsetof(GroundStationHeader, rx_crc_errors) == 20, "rx_crc_errors offset mismatch");
  static_assert(offsetof(GroundStationHeader, rx_size_errors) == 24, "rx_size_errors offset mismatch");
  static_assert(offsetof(GroundStationHeader, tx_reboots) == 28, "tx_reboots offset mismatch");
  static_assert(offsetof(GroundStationHeader, gs_crc) == 30, "gs_crc offset mismatch");
  static_assert(offsetof(UsbTelemetryPacket, rocket) == 32, "rocket offset inside UsbTelemetryPacket mismatch");

  // 2. Verify pinout constants match original ESP32-C3 pinout
  assert(PIN_SCK == 4 && PIN_MISO == 5 && PIN_MOSI == 6 && PIN_CS == 7);
  assert(PIN_RST == 10 && PIN_DIO0 == 3);
  assert(LORA_BANDWIDTH == 250000L && LORA_SPREADING_FACTOR == 7 && TX_INTERVAL_MS == 100UL);

  // 3. Verify CRC-16/CCITT-FALSE standard test vector ("123456789" -> 0x29B1)
  const uint8_t testVec[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  const uint16_t vecCrc = calculateCRC16(testVec, sizeof(testVec));
  assert(vecCrc == 0x29B1U);

  // 4. Verify fault-free bench mode (0 drops, 0 CRC errors across full 290s cycle)
  FlightSimulator cleanSim(0, false);
  for (uint32_t step = 0; step < 2905; step++) {
    LoRaPacket pkt{};
    bool shouldSend = cleanSim.buildNextPacket(step * 100U, pkt);
    assert(shouldSend);
    assert(verifyPacketCrc(pkt));
  }

  // 5. Verify fault-injection mode across 2 full cycles (580 s)
  FlightSimulator sim(0, true);
  int lossWindowSent    = 0;
  int lossWindowDropped = 0;
  int crcInvalidCycle1  = 0;
  int crcInvalidCycle2  = 0;
  bool sawWrapReset     = false;

  FILE* dumpFile = nullptr;
  bool dumpUsb97 = false;
  if (argc >= 3 && std::strcmp(argv[1], "--dump") == 0) {
    dumpFile = std::fopen(argv[2], "wb");
    assert(dumpFile != nullptr);
  } else if (argc >= 3 && std::strcmp(argv[1], "--dump-usb") == 0) {
    dumpFile  = std::fopen(argv[2], "wb");
    dumpUsb97 = true;
    assert(dumpFile != nullptr);
  }

  for (uint32_t step = 0; step < 5805; step++) {
    const uint32_t nowMs = step * 100U;
    LoRaPacket pkt{};
    const bool shouldSend = sim.buildNextPacket(nowMs, pkt);

    if (nowMs >= 165000U && nowMs <= 168000U) {
      if (shouldSend) {
        lossWindowSent++;
      } else {
        lossWindowDropped++;
      }
    }

    if (!shouldSend) {
      continue;
    }

    assert(pkt.sync == SYNC_BYTE);
    const bool crcOk = verifyPacketCrc(pkt);
    if (!crcOk) {
      if (step < 2900) {
        crcInvalidCycle1++;
      } else {
        crcInvalidCycle2++;
      }
    }

    if (step == 2900) {
      assert(pkt.timestamp == 0U);
      assert(pkt.status == STATUS_STANDBY);
      sawWrapReset = true;
    }

    if (dumpFile != nullptr) {
      if (dumpUsb97) {
        UsbTelemetryPacket usb{};
        usb.gs.gs_sync        = GS_SYNC_BYTE;
        usb.gs.gs_flags       = GS_PROTO_VERSION | (crcOk ? 0x01 : 0x00) | 0x02;
        usb.gs.rssi           = -64;
        usb.gs.snr            = 9.25f;
        usb.gs.gs_uptime_ms   = nowMs + 500U;
        usb.gs.rx_packets     = step + 1U;
        usb.gs.rx_lost        = static_cast<uint32_t>(lossWindowDropped);
        usb.gs.rx_crc_errors  = static_cast<uint32_t>(crcInvalidCycle1 + crcInvalidCycle2);
        usb.gs.rx_size_errors = 0U;
        usb.gs.tx_reboots     = (step >= 2900U) ? 1U : 0U;
        usb.gs.gs_crc         = calculateCRC16(reinterpret_cast<const uint8_t*>(&usb.gs), sizeof(GroundStationHeader) - 2);
        usb.rocket            = pkt;
        std::fwrite(&usb, sizeof(usb), 1, dumpFile);
      } else {
        std::fwrite(&pkt, sizeof(pkt), 1, dumpFile);
      }
    }
  }

  if (dumpFile != nullptr) {
    std::fclose(dumpFile);
  }

  // In 165.0..168.0 s (31 ticks at 100 ms), tickCount % 4 == 0 transmits ~8 frames
  // and drops ~23 frames (no odd-parity lockout).
  assert(lossWindowSent >= 7 && lossWindowSent <= 8);
  assert(lossWindowDropped >= 23 && lossWindowDropped <= 24);
  assert(crcInvalidCycle1 == 3);
  assert(crcInvalidCycle2 == 3);
  assert(sawWrapReset);

  // 6. Verify status labels
  assert(std::strcmp(getStatusName(STATUS_STANDBY), "STANDBY") == 0);
  assert(std::strcmp(getStatusName(STATUS_POWERED), "POWERED") == 0);
  assert(std::strcmp(getStatusName(STATUS_APOGEE), "APOGEE") == 0);
  assert(std::strcmp(getStatusName(STATUS_LANDED), "LANDED") == 0);

  std::cout << "[C++ TEST PASSED] 65B layout, CRC-16/CCITT-FALSE, clean & fault modes, and 290s reset verified.\n"
            << "  Loss window (165-168s): sent=" << lossWindowSent
            << " dropped=" << lossWindowDropped << "\n"
            << "  CRC fault injection: cycle1=" << crcInvalidCycle1
            << " cycle2=" << crcInvalidCycle2 << "\n";
  return 0;
}
