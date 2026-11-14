#ifndef LORA_CONFIG_H
#define LORA_CONFIG_H

#include <stdint.h>

// ============================================================================
//  REFERENCE HARDWARE PINOUT & LORA RF CONFIGURATION (ESP32-C3 SUPER MINI)
// ============================================================================
//  Note: Both Arduino sketches (.ino) also expose these constants directly in
//  "SECTION 1: USER CONFIGURATION" so users can edit pins in one place without
//  opening extra header files. This header stays synchronized for shared tests.
// ============================================================================

// --- ESP32-C3 Super Mini -> SX1278 / XL1278-SMT Wiring ---
// Safe FSPI (SPI2) pins + control GPIOs; avoids internal flash GPIOs 12..17
// and boot strapping pins 2, 8, 9.
const int PIN_SCK  = 4;   // SPI Clock
const int PIN_MISO = 5;   // SPI Master In Slave Out
const int PIN_MOSI = 6;   // SPI Master Out Slave In
const int PIN_CS   = 7;   // LoRa NSS / Chip Select
const int PIN_RST  = 10;  // LoRa Reset (set to -1 if disconnected)
const int PIN_DIO0 = 3;   // LoRa DIO0 (Packet Ready / TX Done Interrupt)

// --- LoRa RF Link Parameters (433 MHz, 10 Hz Telemetry Profile) ---
// With SF7, BW = 250 kHz, CR = 4/5, and a 65-byte frame:
//   Time-on-Air (ToA) = 61.57 ms (< 100 ms interval).
const long    LORA_FREQUENCY        = 433E6; // 433 MHz
const long    LORA_BANDWIDTH        = 250E3; // 250 kHz
const int     LORA_SPREADING_FACTOR = 7;     // SF7
const int     LORA_CODING_RATE      = 5;     // CR 4/5
const uint8_t LORA_SYNC_WORD        = 0x12;  // Private sync word

// Safe 2 dBm default for USB bench testing to prevent 3.3V brownout resets.
// Increase to 17..20 dBm ONLY with external 3.3V LDO (>=500 mA) + 470 uF capacitor.
const int           LORA_TX_POWER  = 2;
const unsigned long TX_INTERVAL_MS = 100;

#if defined(ARDUINO)
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

#if defined(ESP_PLATFORM)
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

inline void disableBrownoutDetector() {
#if defined(ESP_PLATFORM) && defined(RTC_CNTL_BROWN_OUT_REG)
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif
}

inline uint8_t readSx1278VersionRegister() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x42 & 0x7F);
  uint8_t version = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  return version;
}

inline bool initLoRaRadio(bool isTransmitter) {
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  LoRa.setPins(PIN_CS, PIN_RST, PIN_DIO0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    return false;
  }

  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();

  if (isTransmitter) {
    LoRa.setTxPower(LORA_TX_POWER);
  }

  return true;
}
#endif // ARDUINO

#endif // LORA_CONFIG_H
