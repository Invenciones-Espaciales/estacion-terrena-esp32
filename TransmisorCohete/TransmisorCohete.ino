#include <SPI.h>
#include <LoRa.h>

#if defined(ESP_PLATFORM)
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

// ============================================================================
//  ROCKET TRANSMITTER (OBC TELEMETRY SIMULATOR - LoRa 433 MHz)
//  Board : ESP32-C3 Super Mini + XL1278-SMT / SX1278 (433 MHz)
//
//  Quick Usage:
//    - Edit pins, TX power, radio settings, or timing in SECTION 1 below.
//    - Transmits the 65-byte Little-Endian LoRaPacket over LoRa at 10 Hz.
//    - Prints a 1 Hz summary line (and instant phase transitions) on Serial.
// ============================================================================


// ============================================================================
//  SECTION 1: USER CONFIGURATION (PINS, RADIO & TIMING)
// ============================================================================

// --- ESP32-C3 Super Mini -> SX1278 / XL1278-SMT Wiring ---
// (Safe FSPI pins + control GPIOs; avoids internal flash GPIOs 12..17)
const int PIN_SCK  = 4;   // SPI Clock
const int PIN_MISO = 5;   // SPI Master In Slave Out
const int PIN_MOSI = 6;   // SPI Master Out Slave In
const int PIN_CS   = 7;   // LoRa NSS / Chip Select
const int PIN_RST  = 10;  // LoRa Reset (set to -1 if RST is disconnected)
const int PIN_DIO0 = 3;   // LoRa DIO0 (Interrupt / TX Done)

// --- LoRa Radio Parameters (Must match Receiver!) ---
const long    LORA_FREQUENCY        = 433E6; // 433 MHz carrier frequency
const long    LORA_BANDWIDTH        = 250E3; // 250 kHz bandwidth (61.6 ms Time-on-Air < 100 ms)
const int     LORA_SPREADING_FACTOR = 7;     // SF7 (fastest chirp rate)
const int     LORA_CODING_RATE      = 5;     // CR 4/5 error coding denominator
const uint8_t LORA_SYNC_WORD        = 0x12;  // Private network sync word
const int     LORA_TX_POWER         = 2;     // 2 dBm safe for USB bench testing!
                                             // Increase to 17..20 dBm ONLY with external 3.3V LDO (>=500 mA) + 470 uF capacitor.

// --- Timing, Telemetry & Simulation Settings ---
const long          SERIAL_BAUD             = 115200;
const unsigned long TX_INTERVAL_MS          = 100;   // 100 ms = 10 Hz transmission cadence
const unsigned long SERIAL_PRINT_MS         = 1000;  // Print 1 summary line per second in Serial Monitor
const bool          ENABLE_SIMULATED_FAULTS = false; // Set true to inject test RF drops (165-168s) & CRC faults (184s)
const bool          DISABLE_BROWNOUT_CHECK  = true;  // Prevents false ESP32-C3 USB brownout resets during TX bursts

// --- Protocol Constants (Must match config/frame.yaml in estacion-terrena-pc) ---
const uint8_t SYNC_BYTE   = 0xAA;
const size_t  PACKET_SIZE = 65;


// ============================================================================
//  SECTION 2: 65-BYTE TELEMETRY PACKET STRUCTURE (LoRaPacket)
// ============================================================================
//  Little-Endian packed struct matching estacion-terrena-pc/config/frame.yaml:
//    Header  [0..6]   : 7 bytes  (sync, frame_number, timestamp)
//    Payload [7..62]  : 56 bytes (status, barometer, IMU, GPS, pyro flags)
//    Trailer [63..64] : 2 bytes  (CRC-16/CCITT-FALSE over bytes 0..62)
// ============================================================================

#define LORA_PACKET_DEFINED
struct __attribute__((packed)) LoRaPacket {
  // Header (7 bytes)
  uint8_t  sync;           // [0]      Fixed sync byte (0xAA)
  uint16_t frame_number;   // [1..2]   Sequential packet counter (0..65535)
  uint32_t timestamp;      // [3..6]   Mission elapsed time (ms)

  // Payload (56 bytes)
  uint8_t  status;         // [7]      Flight stage code (0x00 .. 0xFF)
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

  // Trailer (2 bytes)
  uint16_t crc;            // [63..64] CRC-16/CCITT-FALSE over bytes 0..62
};

static_assert(sizeof(LoRaPacket) == 65, "LoRaPacket size must be exactly 65 bytes!");

#include "flight_simulator.h"


// ============================================================================
//  SECTION 3: GLOBAL VARIABLES
// ============================================================================

// Timing & Simulation State
unsigned long missionStartMs = 0;
unsigned long lastTxMs       = 0;
unsigned long lastSerialMs   = 0;

// Packet Counters
uint16_t frameNumber       = 0;
uint32_t tickCount         = 0;
uint8_t  crcFaultCount     = 0;
uint8_t  lastPrintedStatus = 0xFE;

// Diagnostic Counters
unsigned long txSuccessCount = 0;
unsigned long txDropSimCount = 0;
unsigned long txErrorCount   = 0;


// ============================================================================
//  SECTION 4: HELPER FUNCTIONS (CRC-16, PACKET BUILDER & SERIAL OUTPUT)
// ============================================================================

// Calculates CRC-16/CCITT-FALSE (Poly 0x1021, Init 0xFFFF, No Ref, XorOut 0x0000)
uint16_t calculateCRC16(const uint8_t* data, size_t length) {
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

// Translates raw status byte into a readable rocket mission phase name
const char* getStatusName(uint8_t status) {
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

// Reads SX1278 silicon revision register (0x42, expected 0x12) for wiring debug
uint8_t readChipVersion() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x42 & 0x7F);
  uint8_t version = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  return version;
}

// Builds the next 65-byte telemetry packet. Returns false if intentionally dropped
// during the optional RF dropout test window (when ENABLE_SIMULATED_FAULTS == true).
bool fillTelemetryPacket(LoRaPacket& pkt, unsigned long nowMs) {
  unsigned long elapsedMs = nowMs - missionStartMs;

  // Wrap the 290-second flight simulation back to 0 s
  if (elapsedMs >= SIM_CYCLE_DURATION_MS) {
    missionStartMs = nowMs;
    elapsedMs      = 0;
    crcFaultCount  = 0;
  }

  float timeSeconds    = (float)elapsedMs / 1000.0f;
  uint32_t currentTick = tickCount++;

  // Optional packet-loss test window (165.0s - 168.0s): transmits 1 out of 4 ticks
  if (ENABLE_SIMULATED_FAULTS && timeSeconds >= 165.0f && timeSeconds <= 168.0f) {
    if ((currentTick % 4U) != 0U) {
      frameNumber++;
      return false;
    }
  }

  // 1. Fill 7-byte Header
  pkt.sync         = SYNC_BYTE;
  pkt.frame_number = frameNumber;
  pkt.timestamp    = (uint32_t)elapsedMs;

  // 2. Fill 56-byte Payload
  //    NOTE: To connect REAL sensors later (BMP280, IMU, GPS, pyro pins),
  //    replace the call below with your hardware sensor read functions!
  bool corruptCrc = false;
  fillSimulatedSensorData(pkt, timeSeconds, ENABLE_SIMULATED_FAULTS, corruptCrc);

  // 3. Fill 2-byte Trailer (CRC-16 over first 63 bytes)
  uint16_t crc = calculateCRC16((const uint8_t*)&pkt, PACKET_SIZE - 2);
  if (corruptCrc && crcFaultCount < 3) {
    crc ^= 0xAAAA; // Inject test CRC error
    crcFaultCount++;
  }
  pkt.crc = crc;

  frameNumber++;
  return true;
}

// Prints a single human-readable TX status line in Serial Monitor
void printTransmittedPacket(const LoRaPacket& pkt, unsigned long txDurationMs, bool sentOk) {
  Serial.print("[TX] Frame #"); Serial.print(pkt.frame_number);
  Serial.print(" | T+="); Serial.print(pkt.timestamp / 1000.0f, 1); Serial.print("s");
  Serial.print(" | Phase="); Serial.print(getStatusName(pkt.status));
  Serial.print(" | Alt="); Serial.print(pkt.altura_actual, 1); Serial.print("m");
  Serial.print(" | Az="); Serial.print(pkt.accel_z, 2); Serial.print("g");
  Serial.print(" | ToA="); Serial.print(txDurationMs); Serial.print("ms");
  Serial.print(" | CRC=0x"); Serial.print(pkt.crc, HEX);
  Serial.print(" | Radio="); Serial.println(sentOk ? "OK" : "ERROR");
}


// ============================================================================
//  SECTION 5: SETUP
// ============================================================================

void setup() {
#if defined(ESP_PLATFORM) && defined(RTC_CNTL_BROWN_OUT_REG)
  if (DISABLE_BROWNOUT_CHECK) {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  }
#endif

  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 2500) {
    delay(10);
  }
  delay(300);

  Serial.println();
  Serial.println("====================================================");
  Serial.println("   ROCKET TRANSMITTER - LoRa 433 MHz (ESP32-C3)     ");
  Serial.println("====================================================");

  // Initialize hardware SPI and LoRa module pins using SECTION 1 constants
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  LoRa.setPins(PIN_CS, PIN_RST, PIN_DIO0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.print("ERROR: Failed to initialize SX1278/XL1278! Reg 0x42 = 0x");
    Serial.println(readChipVersion(), HEX);
    Serial.println("Check wiring in SECTION 1 (SCK, MISO, MOSI, CS, RST, DIO0, 3.3V).");
    while (true) {
      delay(1000);
    }
  }

  // Apply RF parameters
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();
  LoRa.setTxPower(LORA_TX_POWER);

  srand((unsigned int)micros());
  unsigned long now = millis();
  missionStartMs = now;
  lastTxMs       = now;
  lastSerialMs   = now;

  Serial.println("LoRa initialized successfully.");
  Serial.print("Radio: 433 MHz | BW 250 kHz | SF 7 | CR 4/5 | Power: ");
  Serial.print(LORA_TX_POWER);
  Serial.println(" dBm");
  Serial.print("Packet size: ");
  Serial.print(sizeof(LoRaPacket));
  Serial.print(" bytes | Rate: 10 Hz (every ");
  Serial.print(TX_INTERVAL_MS);
  Serial.println(" ms)");
  Serial.println("Starting 290-second simulated flight telemetry...");
  Serial.println();
}


// ============================================================================
//  SECTION 6: MAIN LOOP
// ============================================================================

void loop() {
  unsigned long now = millis();

  if (now - lastTxMs >= TX_INTERVAL_MS) {
    lastTxMs += TX_INTERVAL_MS;
    if (now - lastTxMs >= TX_INTERVAL_MS) {
      lastTxMs = now; // Prevent burst catch-up after any single slow iteration
    }

    LoRaPacket txPacket;
    if (!fillTelemetryPacket(txPacket, now)) {
      txDropSimCount++;
      if (now - lastSerialMs >= SERIAL_PRINT_MS) {
        lastSerialMs = now;
        Serial.print("[TX] Simulated RF dropout window | Drops=");
        Serial.println(txDropSimCount);
      }
      return;
    }

    // Send all 65 bytes over LoRa RF
    unsigned long txStart = millis();
    LoRa.beginPacket();
    LoRa.write((const uint8_t*)&txPacket, sizeof(LoRaPacket));
    int result = LoRa.endPacket();
    unsigned long txDurationMs = millis() - txStart;

    bool sentOk = (result == 1);
    if (sentOk) {
      txSuccessCount++;
    } else {
      txErrorCount++;
    }

    // Print status once per second OR immediately when flight phase changes
    bool phaseChanged = (txPacket.status != lastPrintedStatus);
    if (phaseChanged || (now - lastSerialMs >= SERIAL_PRINT_MS)) {
      lastSerialMs      = now;
      lastPrintedStatus = txPacket.status;
      printTransmittedPacket(txPacket, txDurationMs, sentOk);
    }
  }
}
