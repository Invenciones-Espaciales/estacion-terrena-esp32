#ifndef LORA_CONFIG_H
#define LORA_CONFIG_H

#include <stdint.h>

/**
 * @file lora_config.h
 * @brief ESP32-C3 Super Mini hardware pinout, power safety, and SX1278 RF parameters.
 *
 * Uses the native ESP32-C3 hardware FSPI (SPI2) pins (GPIOs 4..7) and safe
 * control GPIOs (3, 10), strictly avoiding internal SPI flash pins (GPIOs 12..17)
 * and strapping pins (GPIOs 2, 8, 9).
 */

// ============================================================
// ESP32-C3 SUPER MINI <-> XL1278-SMT / SX1278 WIRING
// ============================================================
constexpr int PIN_LORA_SCK  = 4;
constexpr int PIN_LORA_MISO = 5;
constexpr int PIN_LORA_MOSI = 6;
constexpr int PIN_LORA_CS   = 7;
constexpr int PIN_LORA_RST  = 10; // Set to -1 if RST is disconnected
constexpr int PIN_LORA_DIO0 = 3;

// ============================================================
// LORA RF LINK PARAMETERS (433 MHz, 10 Hz TELEMETRY PROFILE)
// ============================================================
// With SF7, BW = 250 kHz, CR = 4/5, and a 65-byte binary frame:
//   Time on Air (ToA) = 61.57 ms (< 100 ms period, 47.5% duty cycle).
constexpr long    LORA_FREQUENCY_HZ     = 433E6;
constexpr long    LORA_BANDWIDTH_HZ     = 250E3;
constexpr int     LORA_SPREADING_FACTOR = 7;
constexpr int     LORA_CODING_RATE_4    = 5;
constexpr uint8_t LORA_SYNC_WORD        = 0x12;

// Safe 2 dBm default for USB/LDO bench testing to prevent 3.3V rail brownout resets.
// For outdoor flight range, increase to 17 (or 20) ONLY after adding a 220-470 uF
// bulk decoupling capacitor (or dedicated 3.3V LDO >= 500 mA) on the LoRa VCC rail.
constexpr int LORA_TX_POWER_DBM = 2;

// Nominal 10 Hz transmission cadence (100 ms interval)
constexpr uint32_t TELEMETRY_INTERVAL_MS = 100UL;

#if defined(ARDUINO)
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

#if defined(ESP_PLATFORM)
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

/**
 * @brief Disables the internal ESP32 brownout detector for bench RF testing.
 *
 * Note: For flight hardware, resolve voltage sags electrically using bulk
 * capacitance and a dedicated 3.3V regulator rather than relying on BOD disable.
 */
inline void disableBrownoutDetector() {
#if defined(ESP_PLATFORM) && defined(RTC_CNTL_BROWN_OUT_REG)
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif
}

/**
 * @brief Reads the SX1278 silicon version register (0x42, expected 0x12) via SPI.
 */
inline uint8_t readSx1278VersionRegister() {
    pinMode(PIN_LORA_CS, OUTPUT);
    digitalWrite(PIN_LORA_CS, LOW);
    SPI.transfer(0x42 & 0x7F);
    uint8_t version = SPI.transfer(0x00);
    digitalWrite(PIN_LORA_CS, HIGH);
    return version;
}

/**
 * @brief Initializes FSPI and configures the XL1278-SMT / SX1278 radio.
 * @param isTransmitter True to configure TX power for transmitter role.
 * @return True if the SX1278 responded and configured cleanly.
 */
inline bool initLoRaRadio(bool isTransmitter) {
    // Pass -1 for SS so the LoRa library maintains manual GPIO control over PIN_LORA_CS
    SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, -1);
    LoRa.setPins(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);

    if (!LoRa.begin(LORA_FREQUENCY_HZ)) {
        return false;
    }

    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setSignalBandwidth(LORA_BANDWIDTH_HZ);
    LoRa.setCodingRate4(LORA_CODING_RATE_4);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.enableCrc();

    if (isTransmitter) {
        LoRa.setTxPower(LORA_TX_POWER_DBM);
    }

    return true;
}
#endif // ARDUINO

#endif // LORA_CONFIG_H
