#include <SPI.h>
#include <LoRa.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

#if defined(ESP_PLATFORM)
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

// ============================================================================
//  ROCKET TRANSMITTER - FREERTOS AVIONICS/COMMS EDITION (LoRa 433 MHz)
//  Board : ESP32-C3 Super Mini + XL1278-SMT / SX1278 (433 MHz)
//
//  Architecture (Producer-Consumer Mailbox):
//    [Avionics Team]  TaskAvionics (50 Hz, Prio 3)
//                         │
//                         ▼  calls updateRocketTelemetry(sensorStruct) [~2 us]
//                  [sensorMutex Mailbox]
//                         │
//                         ▼  consumed every 100 ms (non-blocking for Avionics)
//    [COMMS Engine]   TaskLoRaTx (10 Hz, Prio 2) ──[txLogQueue]──▶ TaskSerialLogger (Prio 1)
//
//  Avionics Integration Summary:
//    1. Fill a `LoRaPacket` struct with your 18 sensor/flight fields.
//    2. Pass it to `updateRocketTelemetry(myPacket);` from any task or loop().
//    3. COMMS automatically adds `sync`, `frame_number`, `crc`, and transmits at 10 Hz.
// ============================================================================


// ============================================================================
//  SECTION 1: USER CONFIGURATION (PINS, RADIO, FREERTOS TIMING & PRIORITIES)
// ============================================================================

// --- ESP32-C3 Super Mini -> SX1278 / XL1278-SMT Wiring ---
// (Safe FSPI pins + control GPIOs; avoids internal flash GPIOs 12..17)
const int PIN_SCK  = 4;   // SPI Clock
const int PIN_MISO = 5;   // SPI Master In Slave Out
const int PIN_MOSI = 6;   // SPI Master Out Slave In
const int PIN_CS   = 7;   // LoRa NSS / Chip Select
const int PIN_RST  = 10;  // LoRa Reset (set to -1 if RST is disconnected)
const int PIN_DIO0 = 3;   // LoRa DIO0 (Interrupt / TX Done)

// --- LoRa Radio Parameters (Must match Ground Station Receiver!) ---
const long    LORA_FREQUENCY        = 433E6; // 433 MHz carrier frequency
const long    LORA_BANDWIDTH        = 250E3; // 250 kHz bandwidth (61.6 ms Time-on-Air < 100 ms)
const int     LORA_SPREADING_FACTOR = 7;     // SF7 (fastest chirp rate)
const int     LORA_CODING_RATE      = 5;     // CR 4/5 error coding denominator
const uint8_t LORA_SYNC_WORD        = 0x12;  // Private network sync word
const int     LORA_TX_POWER         = 2;     // 2 dBm safe for USB bench testing!
                                             // Increase to 17..20 dBm ONLY with external 3.3V LDO (>=500 mA) + 470 uF capacitor.

// --- FreeRTOS Task Rates, Priorities & Stacks ---
const unsigned long AVIONICS_INTERVAL_MS = 20;   // 20 ms = 50 Hz Avionics sensor + FSM cadence
const unsigned long TX_INTERVAL_MS       = 100;  // 100 ms = 10 Hz COMMS LoRa transmission
const unsigned long SERIAL_PRINT_MS      = 1000; // 1000 ms = 1 Hz Serial debug summary

const UBaseType_t PRIO_AVIONICS_TASK = 3;        // Highest: Avionics sensor sampling & flight FSM
const UBaseType_t PRIO_LORA_TX_TASK  = 2;        // Medium: COMMS packet framing, CRC-16, SX1278 SPI TX
const UBaseType_t PRIO_SERIAL_TASK   = 1;        // Lowest: USB Serial prints never delay Avionics/LoRa

// Note: On Arduino-ESP32 (ESP-IDF FreeRTOS), task stack size is specified in BYTES.
const uint32_t TASK_STACK_BYTES = 4096;
const size_t   LOG_QUEUE_LENGTH = 12;

// --- General, Simulation & Power Safety Settings ---
const long SERIAL_BAUD             = 115200;
const bool ENABLE_SIMULATED_FAULTS = false; // Set true to inject test RF drops (165-168s) & CRC faults (184s)
const bool DISABLE_BROWNOUT_CHECK  = true;  // Prevents false ESP32-C3 USB brownout resets during TX bursts

// --- Protocol Constants (Must match config/frame.yaml in estacion-terrena-pc) ---
const uint8_t SYNC_BYTE   = 0xAA;
const size_t  PACKET_SIZE = 65;


// ============================================================================
//  SECTION 2: TELEMETRY STRUCT (`LoRaPacket` - 65 BYTES LITTLE-ENDIAN)
// ============================================================================
//  Fields marked [COMMS AUTO] are overwritten automatically by TaskLoRaTx.
//  Fields marked [AVIONICS] are populated by the Avionics team before calling
//  `updateRocketTelemetry(myPacket)`.
// ============================================================================

#define LORA_PACKET_DEFINED
struct __attribute__((packed)) LoRaPacket {
  // --- Header (7 bytes, offsets 0..6) ---
  uint8_t  sync;           // [0]      [COMMS AUTO] Fixed 0xAA sync byte
  uint16_t frame_number;   // [1..2]   [COMMS AUTO] Sequential frame counter
  uint32_t timestamp;      // [3..6]   [AVIONICS]   Mission elapsed time (ms)

  // --- Payload (56 bytes, offsets 7..62) ---
  uint8_t  status;         // [7]      [AVIONICS]   Flight phase code (0x00 .. 0xFF)
  float    pressure;       // [8..11]  [AVIONICS]   Barometric pressure (hPa)
  float    altura_actual;  // [12..15] [AVIONICS]   Barometric altitude (m)
  float    accel_x;        // [16..19] [AVIONICS]   IMU Acceleration X (g)
  float    accel_y;        // [20..23] [AVIONICS]   IMU Acceleration Y (g)
  float    accel_z;        // [24..27] [AVIONICS]   IMU Acceleration Z (g)
  float    gyro_x;         // [28..31] [AVIONICS]   IMU Angular velocity X (deg/s)
  float    gyro_y;         // [32..35] [AVIONICS]   IMU Angular velocity Y (deg/s)
  float    gyro_z;         // [36..39] [AVIONICS]   IMU Angular velocity Z (deg/s)
  float    temp_imu;       // [40..43] [AVIONICS]   IMU Temperature (deg C)
  float    latitude;       // [44..47] [AVIONICS]   GPS Latitude (decimal degrees)
  float    longitude;      // [48..51] [AVIONICS]   GPS Longitude (decimal degrees)
  float    altitud_gps;    // [52..55] [AVIONICS]   GPS Altitude ASL (m)
  uint8_t  satellites;     // [56]     [AVIONICS]   Tracked GPS satellites
  float    ground_speed;   // [57..60] [AVIONICS]   GPS Ground speed (m/s)
  uint8_t  pin_frenado;    // [61]     [AVIONICS]   Drogue chute pyro state (0 = OFF, 1 = ACTIVE)
  uint8_t  pin_principal;  // [62]     [AVIONICS]   Main chute pyro state (0 = OFF, 1 = ACTIVE)

  // --- Trailer (2 bytes, offsets 63..64) ---
  uint16_t crc;            // [63..64] [COMMS AUTO] CRC-16/CCITT-FALSE over bytes 0..62
};

static_assert(sizeof(LoRaPacket) == 65, "LoRaPacket size must be exactly 65 bytes!");

#include "flight_simulator.h"


// ============================================================================
//  SECTION 3: FREERTOS HANDLES & SHARED MAILBOX STATE
// ============================================================================

// Single-slot thread-safe mailbox storing the newest Avionics telemetry snapshot
struct SharedTelemetryMailbox {
  LoRaPacket packet;
  bool       hasData;
};

// Non-blocking event message passed from TaskLoRaTx to TaskSerialLogger
struct TxLogEvent {
  LoRaPacket    packet;
  unsigned long txDurationMs;
  bool          sentOk;
  bool          droppedSim;
  unsigned long dropCount;
};

// FreeRTOS synchronization objects
SemaphoreHandle_t      sensorMutex   = NULL;
QueueHandle_t          txLogQueue    = NULL;
SharedTelemetryMailbox sharedMailbox = {};
unsigned long          bootTimeMs    = 0;

// Task handles (for inspection / stack monitoring)
TaskHandle_t avionicsTaskHandle = NULL;
TaskHandle_t loraTxTaskHandle   = NULL;
TaskHandle_t serialTaskHandle   = NULL;


// ============================================================================
//  SECTION 4: AVIONICS TEAM API (`updateRocketTelemetry`) & AVIONICS TASK
// ============================================================================

/**
 * Single entry point for the Avionics team to hand sensor data to COMMS.
 *
 * Thread-safe and non-blocking (~2 microseconds): copies `newSensorData`
 * into the COMMS mailbox protected by `sensorMutex` (with FreeRTOS priority
 * inheritance) and returns immediately without waiting for the 61.6 ms LoRa TX.
 *
 * @param newSensorData Telemetry struct with the 18 Avionics fields filled.
 *                      (`sync`, `frame_number`, and `crc` are set automatically by COMMS.)
 * @return true if the mailbox was updated cleanly, false if COMMS is not initialized.
 */
bool updateRocketTelemetry(const LoRaPacket& newSensorData) {
  if (sensorMutex == NULL) {
    return false;
  }

  // Wait up to 5 ms so an Avionics flight task can never stall indefinitely
  if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    sharedMailbox.packet  = newSensorData;
    sharedMailbox.hasData = true;
    xSemaphoreGive(sensorMutex);
    return true;
  }

  return false;
}

// --- AVIONICS TASK (Runs at 50 Hz / every 20 ms, Priority 3) ---
// Replace the simulator block inside this task with real BMP280/IMU/GPS reads
// and flight state machine logic when flight sensors are connected.
void TaskAvionics(void* pvParameters) {
  (void)pvParameters;

  TickType_t lastWakeTime = xTaskGetTickCount();
  TickType_t periodTicks  = pdMS_TO_TICKS(AVIONICS_INTERVAL_MS);
  if (periodTicks == 0) {
    periodTicks = 1;
  }

  unsigned long missionStartMs = bootTimeMs;

  while (true) {
    unsigned long nowMs     = millis();
    unsigned long elapsedMs = nowMs - missionStartMs;

    if (elapsedMs >= SIM_CYCLE_DURATION_MS) {
      missionStartMs = nowMs;
      elapsedMs      = 0;
    }

    float timeSeconds = (float)elapsedMs / 1000.0f;

    // 1. Create and populate your telemetry struct
    LoRaPacket avionicsData = {};
    avionicsData.timestamp  = (uint32_t)elapsedMs;

    bool dummyFaultFlag = false;
    fillSimulatedSensorData(avionicsData, timeSeconds, ENABLE_SIMULATED_FAULTS, dummyFaultFlag);

    // 2. Push the struct to COMMS via the single API function (returns in ~2 us)
    updateRocketTelemetry(avionicsData);

    // 3. Sleep deterministically until the next 20 ms sensor cycle
    vTaskDelayUntil(&lastWakeTime, periodTicks);
  }
}


// ============================================================================
//  SECTION 5: COMMS BACKGROUND ENGINE (LoRa TX @ 10 Hz + Serial Logger)
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

// Prints a single human-readable TX status line in Serial Monitor
void printTransmittedPacket(const LoRaPacket& pkt, unsigned long txDurationMs, bool sentOk) {
  Serial.print("[COMMS][RTOS] Frame #"); Serial.print(pkt.frame_number);
  Serial.print(" | T+="); Serial.print(pkt.timestamp / 1000.0f, 1); Serial.print("s");
  Serial.print(" | Phase="); Serial.print(getStatusName(pkt.status));
  Serial.print(" | Alt="); Serial.print(pkt.altura_actual, 1); Serial.print("m");
  Serial.print(" | Az="); Serial.print(pkt.accel_z, 2); Serial.print("g");
  Serial.print(" | ToA="); Serial.print(txDurationMs); Serial.print("ms");
  Serial.print(" | CRC=0x"); Serial.print(pkt.crc, HEX);
  Serial.print(" | Radio="); Serial.println(sentOk ? "OK" : "ERROR");
}

// --- COMMS TASK 1: LoRa Radio Transmitter (10 Hz / every 100 ms, Priority 2) ---
void TaskLoRaTx(void* pvParameters) {
  (void)pvParameters;

  TickType_t lastWakeTime = xTaskGetTickCount();
  TickType_t periodTicks  = pdMS_TO_TICKS(TX_INTERVAL_MS);
  if (periodTicks == 0) {
    periodTicks = 1;
  }

  uint16_t      frameNumber      = 0;
  uint32_t      tickCount        = 0;
  uint32_t      lastTimestampMs  = 0;
  uint8_t       crcFaultCount    = 0;
  unsigned long simDropCount     = 0;

  while (true) {
    // Wait until the next deterministic 100 ms RF slot
    vTaskDelayUntil(&lastWakeTime, periodTicks);

    // Pull latest struct from the Avionics mailbox
    LoRaPacket txPacket = {};
    bool       hasData  = false;

    if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
      hasData  = sharedMailbox.hasData;
      txPacket = sharedMailbox.packet;
      xSemaphoreGive(sensorMutex);
    }

    // Do not transmit empty zeroes before Avionics pushes the first struct
    if (!hasData) {
      continue;
    }

    // Reset per-cycle CRC fault counter if mission timestamp wrapped back to T+0
    if (txPacket.timestamp < lastTimestampMs) {
      crcFaultCount = 0;
    }
    lastTimestampMs = txPacket.timestamp;

    float    timeSeconds = (float)txPacket.timestamp / 1000.0f;
    uint32_t currentTick = tickCount++;

    // Optional RF packet-loss simulation window (165.0s - 168.0s)
    if (ENABLE_SIMULATED_FAULTS && timeSeconds >= 165.0f && timeSeconds <= 168.0f) {
      if ((currentTick % 4U) != 0U) {
        frameNumber++;
        simDropCount++;

        TxLogEvent dropEvent = {};
        dropEvent.droppedSim = true;
        dropEvent.dropCount  = simDropCount;
        xQueueSend(txLogQueue, &dropEvent, 0);
        continue;
      }
    }

    // Stamp COMMS framing fields (sync, sequence counter, and CRC-16)
    txPacket.sync         = SYNC_BYTE;
    txPacket.frame_number = frameNumber;

    uint16_t crc = calculateCRC16((const uint8_t*)&txPacket, PACKET_SIZE - 2);
    if (ENABLE_SIMULATED_FAULTS && timeSeconds >= 184.0f && timeSeconds <= 184.6f && crcFaultCount < 3) {
      crc ^= 0xAAAA;
      crcFaultCount++;
    }
    txPacket.crc = crc;
    frameNumber++;

    // Transmit 65-byte frame over LoRa SX1278
    unsigned long txStart = millis();
    LoRa.beginPacket();
    LoRa.write((const uint8_t*)&txPacket, sizeof(LoRaPacket));
    int result = LoRa.endPacket();
    unsigned long txDurationMs = millis() - txStart;

    // Push non-blocking log event to TaskSerialLogger
    TxLogEvent logEvent = {};
    logEvent.packet       = txPacket;
    logEvent.txDurationMs = txDurationMs;
    logEvent.sentOk       = (result == 1);
    logEvent.droppedSim   = false;
    logEvent.dropCount    = simDropCount;
    xQueueSend(txLogQueue, &logEvent, 0);
  }
}

// --- COMMS TASK 2: USB Serial Logger (Event-Driven, Priority 1) ---
void TaskSerialLogger(void* pvParameters) {
  (void)pvParameters;

  unsigned long lastPrintMs       = millis();
  uint8_t       lastPrintedStatus = 0xFE;
  TxLogEvent    event             = {};

  while (true) {
    if (xQueueReceive(txLogQueue, &event, portMAX_DELAY) == pdTRUE) {
      unsigned long nowMs = millis();

      if (event.droppedSim) {
        if (nowMs - lastPrintMs >= SERIAL_PRINT_MS) {
          lastPrintMs = nowMs;
          Serial.print("[COMMS][RTOS] Simulated RF dropout window | Drops=");
          Serial.println(event.dropCount);
        }
        continue;
      }

      bool phaseChanged = (event.packet.status != lastPrintedStatus);
      if (phaseChanged || (nowMs - lastPrintMs >= SERIAL_PRINT_MS)) {
        lastPrintMs       = nowMs;
        lastPrintedStatus = event.packet.status;
        printTransmittedPacket(event.packet, event.txDurationMs, event.sentOk);
      }
    }
  }
}

// Single initializer for the COMMS subsystem (Radio + Mailbox + Background Tasks)
bool initCommsLoRa() {
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  LoRa.setPins(PIN_CS, PIN_RST, PIN_DIO0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.print("[COMMS] ERROR: Failed to initialize SX1278/XL1278! Reg 0x42 = 0x");
    Serial.println(readChipVersion(), HEX);
    Serial.println("[COMMS] Check wiring in SECTION 1 (SCK, MISO, MOSI, CS, RST, DIO0, 3.3V).");
    return false;
  }

  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();
  LoRa.setTxPower(LORA_TX_POWER);

  sensorMutex = xSemaphoreCreateMutex();
  txLogQueue  = xQueueCreate(LOG_QUEUE_LENGTH, sizeof(TxLogEvent));
  if (sensorMutex == NULL || txLogQueue == NULL) {
    Serial.println("[COMMS] ERROR: Failed to allocate FreeRTOS mutex or queue!");
    return false;
  }

  Serial.println("[COMMS] LoRa 433 MHz initialized (BW 250 kHz | SF7 | CR 4/5).");

  BaseType_t okLoRa   = xTaskCreate(TaskLoRaTx, "TaskLoRaTx", TASK_STACK_BYTES, NULL, PRIO_LORA_TX_TASK, &loraTxTaskHandle);
  BaseType_t okSerial = xTaskCreate(TaskSerialLogger, "TaskSerial", TASK_STACK_BYTES, NULL, PRIO_SERIAL_TASK, &serialTaskHandle);

  if (okLoRa != pdPASS || okSerial != pdPASS) {
    Serial.println("[COMMS] ERROR: Failed to start background COMMS tasks!");
    return false;
  }

  return true;
}


// ============================================================================
//  SECTION 6: SETUP & LOOP
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
  Serial.println("  ROCKET OBC (FreeRTOS Avionics + COMMS) - 433 MHz  ");
  Serial.println("====================================================");

  // 1. Start COMMS subsystem (SX1278 LoRa + Mailbox + TX/Log background tasks)
  if (!initCommsLoRa()) {
    while (true) {
      delay(1000);
    }
  }

  // 2. Initialize Avionics flight task (50 Hz producer)
  srand((unsigned int)micros());
  bootTimeMs = millis();

  BaseType_t okAvionics = xTaskCreate(
    TaskAvionics,
    "TaskAvionics",
    TASK_STACK_BYTES,
    NULL,
    PRIO_AVIONICS_TASK,
    &avionicsTaskHandle
  );
  if (okAvionics != pdPASS) {
    Serial.println("[AVIONICS] ERROR: Failed to create TaskAvionics!");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("[SYSTEM] Running: TaskAvionics (50 Hz) -> updateRocketTelemetry() -> TaskLoRaTx (10 Hz)");
  Serial.println();
}

void loop() {
  // All real-time execution is handled by FreeRTOS tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
