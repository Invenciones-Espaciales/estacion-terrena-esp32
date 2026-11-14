#include <SPI.h>
#include <LoRa.h>

#if defined(ESP_PLATFORM)
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

// ============================================================================
//  GROUND STATION RECEIVER (LoRa 433 MHz -> Serial Monitor / PC Bridge)
//  Board : ESP32-C3 Super Mini + XL1278-SMT / SX1278 (433 MHz)
//
//  Quick Usage:
//    - Edit pins, radio settings, or output mode in SECTION 1 below.
//    - Open Arduino Serial Monitor at 115200 baud to view telemetry cards.
//    - Send 'B' in Serial Monitor to switch to raw 65-byte Binary mode
//      (for estacion-terrena-pc/main.py), or 'H' to return to Human-Readable.
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
const int PIN_DIO0 = 3;   // LoRa DIO0 (Packet Ready Signal)

// --- LoRa Radio Parameters (Must match Transmitter!) ---
const long    LORA_FREQUENCY        = 433E6; // 433 MHz carrier frequency
const long    LORA_BANDWIDTH        = 250E3; // 250 kHz bandwidth (required for 10 Hz link budget)
const int     LORA_SPREADING_FACTOR = 7;     // SF7 (fastest chirp rate, ~61.6 ms Time-on-Air)
const int     LORA_CODING_RATE      = 5;     // CR 4/5 error coding denominator
const uint8_t LORA_SYNC_WORD        = 0x12;  // Private network sync word

// --- Serial, Mode & Display Settings ---
const long          SERIAL_BAUD            = 115200;
const bool          HUMAN_READABLE_MODE    = true;   // true = readable cards, false = raw 65B frames for main.py
const unsigned long PACKET_PRINT_MS        = 500;    // Print detailed card every 500 ms (0 = print every packet)
const unsigned long STATS_INTERVAL_MS      = 1000;   // Print link statistics every 1000 ms
const unsigned long LINK_TIMEOUT_MS        = 2000;   // Mark link INACTIVE after 2000 ms without valid packet
const bool          DISABLE_BROWNOUT_CHECK = true;   // Prevents false ESP32-C3 USB brownout resets

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


// ============================================================================
//  SECTION 3: GLOBAL VARIABLES & LINK STATISTICS
// ============================================================================

bool humanReadableMode = HUMAN_READABLE_MODE;

// Link counters
unsigned long packetsReceived = 0;
unsigned long packetsLost     = 0;
unsigned long crcErrors       = 0;
unsigned long sizeErrors      = 0;
unsigned long txReboots       = 0;

// Sequence & reboot tracking
uint16_t lastFrameNumber   = 0;
uint32_t lastTimestampMs   = 0;
uint8_t  lastPrintedStatus = 0xFE;
bool     hasFirstPacket    = false;

// Timing & RSSI averages
unsigned long lastPacketTimeMs  = 0;
unsigned long lastPacketPrintMs = 0;
unsigned long lastStatsTimeMs   = 0;
long          accumRssi         = 0;
long          rssiSampleCount   = 0;


// ============================================================================
//  SECTION 4: HELPER FUNCTIONS (CRC-16, STATUS LABELS & SERIAL PRINTING)
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
    case 0x00:             return "STANDBY (Pad)";
    case 0x01:             return "POWERED_FLIGHT";
    case 0x02: case 0x03:  return "BURNOUT (Coast)";
    case 0x04: case 0x07:  return "APOGEE";
    case 0x08: case 0x0F:  return "DROGUE_DEPLOY_ORDER";
    case 0x10: case 0x1F:  return "DROGUE_CONFIRMED";
    case 0x20: case 0x3F:  return "MAIN_DEPLOY_ORDER";
    case 0x40: case 0x7F:  return "MAIN_CONFIRMED";
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

// Switches dynamically between Human-Readable ('H') and Binary ('B') over Serial
void checkSerialCommands() {
  while (Serial.available() > 0) {
    int cmd = Serial.read();
    if (cmd == 'b' || cmd == 'B') {
      if (humanReadableMode) {
        Serial.println("\n[MODE] Switching to RAW BINARY mode (65B frames for main.py)...");
        Serial.flush();
        humanReadableMode = false;
      }
    } else if (cmd == 'h' || cmd == 'H') {
      if (!humanReadableMode) {
        humanReadableMode = true;
        Serial.println("\n[MODE] Switched to HUMAN-READABLE debug mode.");
      }
    }
  }
}

// Prints a single decoded telemetry packet in a clean, readable block
void printPacketHumanReadable(const LoRaPacket& pkt, int rssi, float snr, bool crcOk) {
  Serial.println("---------- PACKET RECEIVED ----------");
  Serial.print("Frame #   : "); Serial.print(pkt.frame_number);
  Serial.print("  |  Time: "); Serial.print(pkt.timestamp / 1000.0f, 1); Serial.println(" s");

  Serial.print("Phase     : "); Serial.print(getStatusName(pkt.status));
  Serial.print(" (0x");
  if (pkt.status < 0x10) Serial.print("0");
  Serial.print(pkt.status, HEX);
  Serial.println(")");

  Serial.print("Barometer : Alt="); Serial.print(pkt.altura_actual, 2); Serial.print(" m");
  Serial.print("  |  Pressure="); Serial.print(pkt.pressure, 2); Serial.println(" hPa");

  Serial.print("IMU Accel : X="); Serial.print(pkt.accel_x, 2);
  Serial.print("  Y="); Serial.print(pkt.accel_y, 2);
  Serial.print("  Z="); Serial.print(pkt.accel_z, 2); Serial.println(" g");

  Serial.print("IMU Gyro  : X="); Serial.print(pkt.gyro_x, 1);
  Serial.print("  Y="); Serial.print(pkt.gyro_y, 1);
  Serial.print("  Z="); Serial.print(pkt.gyro_z, 1);
  Serial.print(" deg/s  |  Temp="); Serial.print(pkt.temp_imu, 1); Serial.println(" C");

  Serial.print("GPS       : Lat="); Serial.print(pkt.latitude, 6);
  Serial.print("  Lon="); Serial.print(pkt.longitude, 6);
  Serial.print("  Alt="); Serial.print(pkt.altitud_gps, 1); Serial.println(" m");

  Serial.print("GPS Extra : Sats="); Serial.print(pkt.satellites);
  Serial.print("  Speed="); Serial.print(pkt.ground_speed, 1); Serial.println(" m/s");

  Serial.print("Pyros     : Drogue="); Serial.print(pkt.pin_frenado ? "ACTIVE" : "OFF");
  Serial.print("  |  Main="); Serial.println(pkt.pin_principal ? "ACTIVE" : "OFF");

  Serial.print("Radio RF  : RSSI="); Serial.print(rssi); Serial.print(" dBm");
  Serial.print("  |  SNR="); Serial.print(snr, 1);
  Serial.print(" dB  |  CRC="); Serial.println(crcOk ? "VALID" : "FAILED!");
  Serial.println("-------------------------------------");
}

// Prints 1 Hz link health & packet delivery statistics
void printLinkStatistics() {
  unsigned long now = millis();
  bool linkActive = hasFirstPacket && ((now - lastPacketTimeMs) < LINK_TIMEOUT_MS);

  unsigned long totalExpected = packetsReceived + packetsLost;
  float pdr = 0.0f;
  if (totalExpected > 0) {
    pdr = (100.0f * packetsReceived) / totalExpected;
  }

  float avgRssi = 0.0f;
  if (rssiSampleCount > 0) {
    avgRssi = (float)accumRssi / (float)rssiSampleCount;
  }

  Serial.println();
  Serial.println("========== STATISTICS ==========");
  Serial.print("Link Active   : "); Serial.println(linkActive ? "YES" : "NO");
  Serial.print("Packets RX    : "); Serial.println(packetsReceived);
  Serial.print("Packets Lost  : "); Serial.println(packetsLost);
  Serial.print("CRC Errors    : "); Serial.println(crcErrors);
  Serial.print("Size Errors   : "); Serial.println(sizeErrors);
  Serial.print("TX Reboots    : "); Serial.println(txReboots);
  Serial.print("PDR           : "); Serial.print(pdr, 2); Serial.println(" %");
  Serial.print("Average RSSI  : "); Serial.print(avgRssi, 1); Serial.println(" dBm");
  Serial.println("Tip: Send 'B' for binary mode (main.py), 'H' for human-readable");
  Serial.println("================================");
  Serial.println();
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

  if (humanReadableMode) {
    Serial.println();
    Serial.println("====================================================");
    Serial.println("   GROUND STATION RECEIVER - LoRa 433 MHz (ESP32-C3)");
    Serial.println("====================================================");
  }

  // Initialize hardware SPI and LoRa module pins using SECTION 1 constants
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  LoRa.setPins(PIN_CS, PIN_RST, PIN_DIO0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    if (humanReadableMode) {
      Serial.print("ERROR: Failed to initialize SX1278/XL1278! Reg 0x42 = 0x");
      Serial.println(readChipVersion(), HEX);
      Serial.println("Check wiring in SECTION 1 (SCK, MISO, MOSI, CS, RST, DIO0, 3.3V).");
    }
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

  lastStatsTimeMs = millis();

  if (humanReadableMode) {
    Serial.println("LoRa initialized successfully.");
    Serial.println("Radio: 433 MHz | BW 250 kHz | SF 7 | CR 4/5 | Sync 0x12");
    Serial.println("Listening for 65-byte LoRaPacket frames...");
    Serial.println("Send 'B' over Serial for raw binary output (estacion-terrena-pc/main.py).");
    Serial.println();
  }
}


// ============================================================================
//  SECTION 6: MAIN LOOP
// ============================================================================

void loop() {
  checkSerialCommands();

  int packetSize = LoRa.parsePacket();
  unsigned long now = millis();

  if (packetSize > 0) {
    // 1. Verify expected 65-byte frame length
    if (packetSize != (int)PACKET_SIZE) {
      sizeErrors++;
      while (LoRa.available()) {
        LoRa.read();
      }
      if (humanReadableMode) {
        Serial.print("[WARN] Unexpected packet size: ");
        Serial.print(packetSize);
        Serial.print(" bytes (expected ");
        Serial.print(PACKET_SIZE);
        Serial.println(")");
      }
      return;
    }

    // 2. Read raw bytes straight into our 65-byte LoRaPacket struct
    LoRaPacket rxPacket;
    size_t bytesRead = LoRa.readBytes((uint8_t*)&rxPacket, PACKET_SIZE);
    if (bytesRead != PACKET_SIZE) {
      sizeErrors++;
      return;
    }

    // 3. Check 0xAA sync header byte
    if (rxPacket.sync != SYNC_BYTE) {
      sizeErrors++;
      if (humanReadableMode) {
        Serial.print("[WARN] Invalid sync byte: 0x");
        Serial.println(rxPacket.sync, HEX);
      }
      return;
    }

    // 4. In RAW BINARY mode, forward valid frames directly to PC software (main.py)
    if (!humanReadableMode) {
      Serial.write((const uint8_t*)&rxPacket, PACKET_SIZE);
      packetsReceived++;
      lastPacketTimeMs = now;
      hasFirstPacket   = true;
      return;
    }

    // 5. Verify CRC-16/CCITT-FALSE over bytes 0..62
    uint16_t expectedCrc = calculateCRC16((const uint8_t*)&rxPacket, PACKET_SIZE - 2);
    bool crcOk = (expectedCrc == rxPacket.crc);

    int rssi  = LoRa.packetRssi();
    float snr = LoRa.packetSnr();

    if (!crcOk) {
      crcErrors++;
      printPacketHumanReadable(rxPacket, rssi, snr, false);
      return;
    }

    // 6. Track packet sequence gaps and transmitter reboots/cycles
    if (hasFirstPacket) {
      bool timeReset  = (rxPacket.timestamp < lastTimestampMs);
      bool frameReset = (lastFrameNumber > 20 && rxPacket.frame_number < 10);

      if (timeReset && frameReset) {
        txReboots++;
        Serial.println("[INFO] Transmitter reboot or 290s cycle restart detected.");
      } else {
        uint16_t delta = (uint16_t)(rxPacket.frame_number - lastFrameNumber);
        if (delta > 1 && delta < 32768) {
          unsigned long gap = (unsigned long)(delta - 1);
          packetsLost += gap;
          Serial.print("[WARN] Packet loss detected: ");
          Serial.print(gap);
          Serial.println(" frame(s) missed.");
        }
      }
    }

    lastFrameNumber  = rxPacket.frame_number;
    lastTimestampMs  = rxPacket.timestamp;
    lastPacketTimeMs = now;
    hasFirstPacket   = true;

    packetsReceived++;
    accumRssi += rssi;
    rssiSampleCount++;

    // 7. Print decoded telemetry card (throttled by PACKET_PRINT_MS, immediate on phase change)
    bool phaseChanged = (rxPacket.status != lastPrintedStatus);
    if (PACKET_PRINT_MS == 0 || phaseChanged || (now - lastPacketPrintMs >= PACKET_PRINT_MS)) {
      lastPacketPrintMs = now;
      lastPrintedStatus = rxPacket.status;
      printPacketHumanReadable(rxPacket, rssi, snr, true);
    }
  }

  // Print periodic 1 Hz statistics in Human-Readable mode
  if (humanReadableMode && (now - lastStatsTimeMs >= STATS_INTERVAL_MS)) {
    lastStatsTimeMs = now;
    printLinkStatistics();
  }
}
