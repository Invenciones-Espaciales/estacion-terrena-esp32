#include <SPI.h>
#include <LoRa.h>

#if defined(ESP_PLATFORM)
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

// ============================================================================
//  GROUND STATION RECEIVER (LoRa 433 MHz -> 'H' Debug / 97B 'B' PC Bridge)
//  Board : ESP32-C3 Super Mini + XL1278-SMT / SX1278 (433 MHz)
//
//  Output Modes (Live Mode Router):
//    - 'H' (Default at boot) : Human-Readable telemetry cards + 1 Hz stats.
//    - 'B' (PC Dashboard)    : 97-byte Binary USB frames (32B GS Header + 65B
//                              Rocket Frame). Requires 'B' keepalive every
//                              2 s; auto-reverts to 'H' after 6 s if PC dies.
//    - 'M' (Manual Binary)   : Sticky 97-byte Binary mode (no timeout) until 'H'.
// ============================================================================


// ============================================================================
//  SECTION 1: USER CONFIGURATION (PINS, RADIO, WATCHDOG & DISPLAY)
// ============================================================================

// --- ESP32-C3 Super Mini -> SX1278 / XL1278-SMT Wiring ---
// (Safe FSPI pins + control GPIOs; avoids internal flash GPIOs 12..17)
const int PIN_SCK  = 4;   // SPI Clock
const int PIN_MISO = 5;   // SPI Master In Slave Out
const int PIN_MOSI = 6;   // SPI Master Out Slave In
const int PIN_CS   = 7;   // LoRa NSS / Chip Select
const int PIN_RST  = 10;  // LoRa Reset (set to -1 if RST is disconnected)
const int PIN_DIO0 = 3;   // LoRa DIO0 (Packet Ready Signal)

// --- LoRa Radio Parameters (Must match Rocket Transmitter!) ---
const long    LORA_FREQUENCY        = 433E6; // 433 MHz carrier frequency
const long    LORA_BANDWIDTH        = 250E3; // 250 kHz bandwidth (10 Hz link budget)
const int     LORA_SPREADING_FACTOR = 7;     // SF7 (fastest chirp rate, ~61.6 ms Time-on-Air)
const int     LORA_CODING_RATE      = 5;     // CR 4/5 error coding denominator
const uint8_t LORA_SYNC_WORD        = 0x12;  // Private network sync word

// --- Serial, Mode Router & Timing Settings ---
const long          SERIAL_BAUD            = 115200;
const bool          HUMAN_READABLE_DEFAULT = true;   // Always start in 'H' mode on boot
const unsigned long PACKET_PRINT_MS        = 500;    // Detailed card cadence in 'H' mode (0 = every packet)
const unsigned long STATS_INTERVAL_MS      = 1000;   // Link stats print cadence in 'H' mode
const unsigned long LINK_TIMEOUT_MS        = 2000;   // Link marked inactive after 2.0 s of RF silence
const unsigned long BINARY_WATCHDOG_MS     = 6000;   // Auto-revert 'B' -> 'H' after 6.0 s without keepalive
const bool          DISABLE_BROWNOUT_CHECK = true;   // Prevents false ESP32-C3 USB brownout resets

// --- Protocol Constants ---
const uint8_t GS_SYNC_BYTE       = 0xAB; // Ground Station USB header sync byte
const uint8_t GS_PROTO_VERSION   = 0x10; // Upper nibble = protocol v1
const uint8_t ROCKET_SYNC_BYTE   = 0xAA; // Rocket RF frame sync byte (config/frame.yaml)
const size_t  ROCKET_PACKET_SIZE = 65;   // Fixed 65-byte over-the-air LoRa frame


// ============================================================================
//  SECTION 2: BINARY TELEMETRY STRUCTURES (32B GS HEADER + 65B ROCKET = 97B)
// ============================================================================
//  Over the air (RF), only the 65-byte `LoRaPacket` is transmitted.
//  Over USB Serial in Binary mode, the Ground Station prepends a fixed
//  32-byte `GroundStationHeader` containing RSSI, SNR, and RX link counters.
// ============================================================================

struct __attribute__((packed)) GroundStationHeader {
  uint8_t  gs_sync;        // [0]      Fixed GS header sync byte (0xAB)
  uint8_t  gs_flags;       // [1]      Bits 7..4 = proto v1 (0x10);
                           //          bit0=rocket_crc_ok, bit1=link_active, bit2=tx_reboot
  int16_t  rssi;           // [2..3]   LoRa packet RSSI (dBm)
  float    snr;            // [4..7]   LoRa packet SNR (dB)
  uint32_t gs_uptime_ms;   // [8..11]  Ground Station uptime millis()
  uint32_t rx_packets;     // [12..15] Valid 65B rocket packets received
  uint32_t rx_lost;        // [16..19] Estimated lost packets (sequence gaps)
  uint32_t rx_crc_errors;  // [20..23] Software CRC-16 failure counter
  uint32_t rx_size_errors; // [24..27] Unexpected packet length / sync error counter
  uint16_t tx_reboots;     // [28..29] Detected Rocket TX resets / 290s cycle wraps
  uint16_t gs_crc;         // [30..31] CRC-16/CCITT-FALSE over GS header bytes 0..29
};

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

struct __attribute__((packed)) UsbTelemetryPacket {
  GroundStationHeader gs;     // [0..31]  32 bytes GS metadata + GS CRC
  LoRaPacket          rocket; // [32..96] 65 bytes untouched Rocket RF frame
};

static_assert(sizeof(GroundStationHeader) == 32, "GroundStationHeader must be 32 bytes!");
static_assert(sizeof(LoRaPacket) == 65,          "LoRaPacket must be 65 bytes!");
static_assert(sizeof(UsbTelemetryPacket) == 97,  "UsbTelemetryPacket must be 97 bytes!");

enum RxOutputMode {
  MODE_HUMAN_READABLE  = 0,
  MODE_BINARY_WATCHDOG = 1,
  MODE_BINARY_STICKY   = 2
};


// ============================================================================
//  SECTION 3: GLOBAL VARIABLES & LINK COUNTERS
// ============================================================================

RxOutputMode  rxMode                = HUMAN_READABLE_DEFAULT ? MODE_HUMAN_READABLE : MODE_BINARY_WATCHDOG;
unsigned long lastBinaryKeepaliveMs = 0;

// Persistent link integrity & diagnostics (active in BOTH 'H' and 'B' modes)
uint32_t packetsReceived = 0;
uint32_t packetsLost     = 0;
uint32_t crcErrors       = 0;
uint32_t sizeErrors      = 0;
uint16_t txReboots       = 0;

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
//  SECTION 4: HELPER FUNCTIONS (CRC-16, STATUS LABELS & LIVE MODE ROUTER)
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

// Handles live 'B' (dashboard watchdog binary), 'M' (manual sticky binary),
// 'H' (human-readable), and auto-fallback when dashboard keepalives stop.
void updateModeRouter(unsigned long now) {
  while (Serial.available() > 0) {
    int cmd = Serial.read();

    if (cmd == 'b' || cmd == 'B') {
      lastBinaryKeepaliveMs = now;
      if (rxMode == MODE_HUMAN_READABLE) {
        Serial.println("\n[MODE] Switching to BINARY mode (97B GS+Rocket frames)...");
        Serial.flush();
        rxMode = MODE_BINARY_WATCHDOG;
      } else if (rxMode == MODE_BINARY_STICKY) {
        rxMode = MODE_BINARY_WATCHDOG;
      }
    } else if (cmd == 'm' || cmd == 'M') {
      if (rxMode == MODE_HUMAN_READABLE) {
        Serial.println("\n[MODE] Switching to STICKY BINARY mode (no timeout, send 'H' to exit)...");
        Serial.flush();
      }
      rxMode = MODE_BINARY_STICKY;
    } else if (cmd == 'h' || cmd == 'H') {
      if (rxMode != MODE_HUMAN_READABLE) {
        rxMode          = MODE_HUMAN_READABLE;
        lastStatsTimeMs = now;
        Serial.println("\n[MODE] Switched to HUMAN-READABLE debug mode.");
      }
    }
  }

  // Watchdog fallback: if dashboard crashes or USB disconnects without sending 'H'
  if (rxMode == MODE_BINARY_WATCHDOG && BINARY_WATCHDOG_MS > 0) {
    if (now - lastBinaryKeepaliveMs >= BINARY_WATCHDOG_MS) {
      rxMode          = MODE_HUMAN_READABLE;
      lastStatsTimeMs = now;
      Serial.println("\n[MODE] Dashboard keepalive timeout (6.0 s). Reverted to HUMAN-READABLE mode.");
    }
  }
}


// ============================================================================
//  SECTION 5: PACKET PROCESSING, BINARY BUILDER & HUMAN-READABLE FORMATTERS
// ============================================================================

// Tracks sequence gaps & TX reboots/290s cycle wraps in BOTH 'H' and 'B' modes.
// Returns true if this packet marks a TX reboot or cycle restart.
bool updateSequenceStats(const LoRaPacket& pkt, unsigned long now) {
  bool txRebootEvent = false;

  if (hasFirstPacket) {
    bool timeReset  = (pkt.timestamp < lastTimestampMs);
    bool frameReset = (lastFrameNumber > 20 && pkt.frame_number < 10);

    if (timeReset && frameReset) {
      txReboots++;
      txRebootEvent = true;
      if (rxMode == MODE_HUMAN_READABLE) {
        Serial.println("[INFO] Transmitter reboot or 290s cycle restart detected.");
      }
    } else {
      uint16_t delta = (uint16_t)(pkt.frame_number - lastFrameNumber);
      if (delta > 1 && delta < 32768) {
        uint32_t gap = (uint32_t)(delta - 1);
        packetsLost += gap;
        if (rxMode == MODE_HUMAN_READABLE) {
          Serial.print("[WARN] Packet loss detected: ");
          Serial.print(gap);
          Serial.println(" frame(s) missed.");
        }
      }
    }
  }

  lastFrameNumber  = pkt.frame_number;
  lastTimestampMs  = pkt.timestamp;
  lastPacketTimeMs = now;
  hasFirstPacket   = true;
  return txRebootEvent;
}

// Builds the 97-byte USB frame (32B GS metadata + 65B untouched Rocket frame)
UsbTelemetryPacket buildUsbTelemetryPacket(
    const LoRaPacket& rocketPkt,
    int               rssi,
    float             snr,
    bool              rocketCrcOk,
    bool              txRebootEvent,
    unsigned long     now
) {
  UsbTelemetryPacket out = {};

  bool linkActive = hasFirstPacket && ((now - lastPacketTimeMs) < LINK_TIMEOUT_MS);
  uint8_t flags   = GS_PROTO_VERSION;
  if (rocketCrcOk)   flags |= 0x01;
  if (linkActive)    flags |= 0x02;
  if (txRebootEvent) flags |= 0x04;

  out.gs.gs_sync        = GS_SYNC_BYTE;
  out.gs.gs_flags       = flags;
  out.gs.rssi           = (int16_t)rssi;
  out.gs.snr            = snr;
  out.gs.gs_uptime_ms   = (uint32_t)now;
  out.gs.rx_packets     = packetsReceived;
  out.gs.rx_lost        = packetsLost;
  out.gs.rx_crc_errors  = crcErrors;
  out.gs.rx_size_errors = sizeErrors;
  out.gs.tx_reboots     = txReboots;
  out.gs.gs_crc         = calculateCRC16((const uint8_t*)&out.gs, sizeof(GroundStationHeader) - 2);

  out.rocket = rocketPkt;
  return out;
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
  Serial.println("Commands: 'B'=Binary (97B w/ 6s watchdog), 'M'=Sticky Binary, 'H'=Human");
  Serial.println("================================");
  Serial.println();
}


// ============================================================================
//  SECTION 6: SETUP
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

  if (rxMode == MODE_HUMAN_READABLE) {
    Serial.println();
    Serial.println("====================================================");
    Serial.println("   GROUND STATION RECEIVER - LoRa 433 MHz (ESP32-C3)");
    Serial.println("====================================================");
  }

  // Initialize hardware SPI and LoRa module pins using SECTION 1 constants
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  LoRa.setPins(PIN_CS, PIN_RST, PIN_DIO0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    if (rxMode == MODE_HUMAN_READABLE) {
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

  unsigned long now     = millis();
  lastStatsTimeMs       = now;
  lastBinaryKeepaliveMs = now;

  if (rxMode == MODE_HUMAN_READABLE) {
    Serial.println("LoRa initialized successfully.");
    Serial.println("Radio: 433 MHz | BW 250 kHz | SF 7 | CR 4/5 | Sync 0x12");
    Serial.println("Mode : HUMAN-READABLE ('H'). Send 'B' for 97B PC Binary mode.");
    Serial.println();
  }
}


// ============================================================================
//  SECTION 7: MAIN LOOP
// ============================================================================

void loop() {
  unsigned long now = millis();
  updateModeRouter(now);

  int packetSize = LoRa.parsePacket();

  if (packetSize > 0) {
    // 1. Validate 65-byte RF air packet length
    if (packetSize != (int)ROCKET_PACKET_SIZE) {
      sizeErrors++;
      while (LoRa.available()) {
        LoRa.read();
      }
      if (rxMode == MODE_HUMAN_READABLE) {
        Serial.print("[WARN] Unexpected packet size: ");
        Serial.print(packetSize);
        Serial.print(" bytes (expected ");
        Serial.print(ROCKET_PACKET_SIZE);
        Serial.println(")");
      }
    } else {
      // 2. Read raw 65 bytes straight into LoRaPacket
      LoRaPacket rxPacket = {};
      size_t bytesRead = LoRa.readBytes((uint8_t*)&rxPacket, ROCKET_PACKET_SIZE);

      if (bytesRead != ROCKET_PACKET_SIZE || rxPacket.sync != ROCKET_SYNC_BYTE) {
        sizeErrors++;
        if (rxMode == MODE_HUMAN_READABLE) {
          Serial.print("[WARN] Invalid sync/read | len=");
          Serial.print(bytesRead);
          Serial.print(" sync=0x");
          Serial.println(rxPacket.sync, HEX);
        }
      } else {
        // 3. Verify Rocket CRC-16/CCITT-FALSE & read RF metrics
        uint16_t expectedCrc = calculateCRC16((const uint8_t*)&rxPacket, ROCKET_PACKET_SIZE - 2);
        bool crcOk           = (expectedCrc == rxPacket.crc);
        int rssi             = LoRa.packetRssi();
        float snr            = LoRa.packetSnr();
        bool txRebootEvent   = false;

        if (crcOk) {
          packetsReceived++;
          accumRssi += rssi;
          rssiSampleCount++;
          txRebootEvent = updateSequenceStats(rxPacket, now);
        } else {
          crcErrors++;
        }

        // 4. Emit 97B GS+Rocket frame in Binary mode OR cards in Human-Readable mode
        if (rxMode != MODE_HUMAN_READABLE) {
          UsbTelemetryPacket usbPacket = buildUsbTelemetryPacket(
            rxPacket,
            rssi,
            snr,
            crcOk,
            txRebootEvent,
            now
          );
          Serial.write((const uint8_t*)&usbPacket, sizeof(UsbTelemetryPacket));
        } else {
          bool phaseChanged = crcOk && (rxPacket.status != lastPrintedStatus);
          if (!crcOk || PACKET_PRINT_MS == 0 || phaseChanged || (now - lastPacketPrintMs >= PACKET_PRINT_MS)) {
            lastPacketPrintMs = now;
            if (crcOk) {
              lastPrintedStatus = rxPacket.status;
            }
            printPacketHumanReadable(rxPacket, rssi, snr, crcOk);
          }
        }
      }
    }
  }

  // 5. Print periodic 1 Hz statistics in Human-Readable mode
  if (rxMode == MODE_HUMAN_READABLE && (now - lastStatsTimeMs >= STATS_INTERVAL_MS)) {
    lastStatsTimeMs = now;
    printLinkStatistics();
  }
}
