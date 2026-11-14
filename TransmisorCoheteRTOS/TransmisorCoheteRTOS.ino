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
//  ROCKET TRANSMITTER - FREERTOS EDITION (OBC TELEMETRY SIMULATOR - LoRa 433 MHz)
//  Board : ESP32-C3 Super Mini + XL1278-SMT / SX1278 (433 MHz)
//
//  Architecture (3 FreeRTOS Tasks):
//    1. TaskSensors      (50 Hz, Priority 3) -> Acquires sensor / flight state
//    2. TaskLoRaTx       (10 Hz, Priority 2) -> Packs 65B frame + CRC + sends LoRa
//    3. TaskSerialLogger (Event, Priority 1) -> Prints 1 Hz summary over USB Serial
//
//  Quick Usage:
//    - Edit pins, TX power, radio settings, or task rates in SECTION 1 below.
//    - Uses the same 65-byte Little-Endian protocol as TransmisorCohete.ino.
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

// --- LoRa Radio Parameters (Must match Receiver!) ---
const long    LORA_FREQUENCY        = 433E6; // 433 MHz carrier frequency
const long    LORA_BANDWIDTH        = 250E3; // 250 kHz bandwidth (61.6 ms Time-on-Air < 100 ms)
const int     LORA_SPREADING_FACTOR = 7;     // SF7 (fastest chirp rate)
const int     LORA_CODING_RATE      = 5;     // CR 4/5 error coding denominator
const uint8_t LORA_SYNC_WORD        = 0x12;  // Private network sync word
const int     LORA_TX_POWER         = 2;     // 2 dBm safe for USB bench testing!
                                             // Increase to 17..20 dBm ONLY with external 3.3V LDO (>=500 mA) + 470 uF capacitor.

// --- FreeRTOS Task Rates, Priorities & Stacks ---
const unsigned long SENSOR_INTERVAL_MS = 20;   // 20 ms = 50 Hz sensor acquisition loop
const unsigned long TX_INTERVAL_MS     = 100;  // 100 ms = 10 Hz LoRa radio transmission
const unsigned long SERIAL_PRINT_MS    = 1000; // 1000 ms = 1 Hz Serial summary (plus instant phase changes)

const UBaseType_t PRIO_SENSOR_TASK  = 3;       // Highest: deterministic sensor sampling
const UBaseType_t PRIO_LORA_TX_TASK = 2;       // Medium: 65B framing, CRC-16, SPI LoRa TX
const UBaseType_t PRIO_SERIAL_TASK  = 1;       // Lowest: USB Serial prints never block radio/sensors

// Note: On Arduino-ESP32 (ESP-IDF FreeRTOS), task stack size is specified in BYTES.
const uint32_t TASK_STACK_BYTES  = 4096;
const size_t   LOG_QUEUE_LENGTH  = 12;

// --- General, Simulation & Power Safety Settings ---
const long SERIAL_BAUD             = 115200;
const bool ENABLE_SIMULATED_FAULTS = false; // Set true to inject test RF drops (165-168s) & CRC faults (184s)
const bool DISABLE_BROWNOUT_CHECK  = true;  // Prevents false ESP32-C3 USB brownout resets during TX bursts

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
//  SECTION 3: FREERTOS HANDLES, SHARED DATA & LOG EVENT STRUCT
// ============================================================================

// Latest sensor reading produced by TaskSensors (50 Hz) and read by TaskLoRaTx (10 Hz)
struct SharedTelemetry {
  LoRaPacket packet;
  bool       corruptCrc;
  uint32_t   cycleCount;
};

// Telemetry log message sent from TaskLoRaTx to TaskSerialLogger
struct TxLogEvent {
  LoRaPacket    packet;
  unsigned long txDurationMs;
  bool          sentOk;
  bool          droppedSim;
  unsigned long dropCount;
};

// Synchronization primitives
SemaphoreHandle_t sensorMutex = NULL;
QueueHandle_t     txLogQueue  = NULL;

// Shared state protected by sensorMutex
SharedTelemetry sharedTelemetry = {};
unsigned long   bootTimeMs      = 0;

// Task handles (useful for monitoring / stack inspection)
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t loraTxTaskHandle = NULL;
TaskHandle_t serialTaskHandle = NULL;


// ============================================================================
//  SECTION 4: HELPER FUNCTIONS (CRC-16, STATUS LABELS & SERIAL OUTPUT)
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
  Serial.print("[TX][RTOS] Frame #"); Serial.print(pkt.frame_number);
  Serial.print(" | T+="); Serial.print(pkt.timestamp / 1000.0f, 1); Serial.print("s");
  Serial.print(" | Phase="); Serial.print(getStatusName(pkt.status));
  Serial.print(" | Alt="); Serial.print(pkt.altura_actual, 1); Serial.print("m");
  Serial.print(" | Az="); Serial.print(pkt.accel_z, 2); Serial.print("g");
  Serial.print(" | ToA="); Serial.print(txDurationMs); Serial.print("ms");
  Serial.print(" | CRC=0x"); Serial.print(pkt.crc, HEX);
  Serial.print(" | Radio="); Serial.println(sentOk ? "OK" : "ERROR");
}


// ============================================================================
//  SECTION 5: FREERTOS TASKS
// ============================================================================

// --- TASK 1: High-Priority Sensor Acquisition (50 Hz / every 20 ms) ---
void TaskSensors(void* pvParameters) {
  (void)pvParameters;

  TickType_t lastWakeTime = xTaskGetTickCount();
  TickType_t periodTicks  = pdMS_TO_TICKS(SENSOR_INTERVAL_MS);
  if (periodTicks == 0) {
    periodTicks = 1;
  }

  unsigned long missionStartMs = bootTimeMs;
  uint32_t      cycleCount     = 0;

  while (true) {
    unsigned long nowMs     = millis();
    unsigned long elapsedMs = nowMs - missionStartMs;

    // Wrap the 290-second simulated mission back to T+0
    if (elapsedMs >= SIM_CYCLE_DURATION_MS) {
      missionStartMs = nowMs;
      elapsedMs      = 0;
      cycleCount++;
    }

    float timeSeconds = (float)elapsedMs / 1000.0f;

    LoRaPacket localPacket = {};
    localPacket.sync      = SYNC_BYTE;
    localPacket.timestamp = (uint32_t)elapsedMs;

    // NOTE: To integrate REAL sensors later (BMP280, IMU, GPS, pyro sense),
    // replace fillSimulatedSensorData(...) below with your sensor driver reads!
    bool corruptCrc = false;
    fillSimulatedSensorData(localPacket, timeSeconds, ENABLE_SIMULATED_FAULTS, corruptCrc);

    // Publish latest telemetry snapshot safely under mutex
    if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
      sharedTelemetry.packet     = localPacket;
      sharedTelemetry.corruptCrc = corruptCrc;
      sharedTelemetry.cycleCount = cycleCount;
      xSemaphoreGive(sensorMutex);
    }

    vTaskDelayUntil(&lastWakeTime, periodTicks);
  }
}

// --- TASK 2: Medium-Priority LoRa Radio Transmitter (10 Hz / every 100 ms) ---
void TaskLoRaTx(void* pvParameters) {
  (void)pvParameters;

  TickType_t lastWakeTime = xTaskGetTickCount();
  TickType_t periodTicks  = pdMS_TO_TICKS(TX_INTERVAL_MS);
  if (periodTicks == 0) {
    periodTicks = 1;
  }

  uint16_t      frameNumber   = 0;
  uint32_t      tickCount     = 0;
  uint32_t      lastSeenCycle = 0;
  uint8_t       crcFaultCount = 0;
  unsigned long simDropCount  = 0;

  while (true) {
    // Wait precisely until the next 100 ms RF slot
    vTaskDelayUntil(&lastWakeTime, periodTicks);

    // Copy the latest sensor snapshot under mutex (keep critical section tiny)
    SharedTelemetry snapshot = {};
    if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
      snapshot = sharedTelemetry;
      xSemaphoreGive(sensorMutex);
    }

    // Reset per-cycle simulated fault counter when 290s mission restarts
    if (snapshot.cycleCount != lastSeenCycle) {
      lastSeenCycle = snapshot.cycleCount;
      crcFaultCount = 0;
    }

    LoRaPacket txPacket = snapshot.packet;
    float timeSeconds   = (float)txPacket.timestamp / 1000.0f;
    uint32_t currentTick = tickCount++;

    // Optional packet-loss test window (165.0s - 168.0s): transmits 1 out of 4 ticks
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

    // Stamp header sequence & compute CRC-16 over first 63 bytes
    txPacket.sync         = SYNC_BYTE;
    txPacket.frame_number = frameNumber;

    uint16_t crc = calculateCRC16((const uint8_t*)&txPacket, PACKET_SIZE - 2);
    if (snapshot.corruptCrc && crcFaultCount < 3) {
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

    // Enqueue log event without blocking RF timing
    TxLogEvent logEvent = {};
    logEvent.packet       = txPacket;
    logEvent.txDurationMs = txDurationMs;
    logEvent.sentOk       = (result == 1);
    logEvent.droppedSim   = false;
    logEvent.dropCount    = simDropCount;
    xQueueSend(txLogQueue, &logEvent, 0);
  }
}

// --- TASK 3: Low-Priority USB Serial Logger (Blocks until event arrives) ---
void TaskSerialLogger(void* pvParameters) {
  (void)pvParameters;

  unsigned long lastPrintMs       = millis();
  uint8_t       lastPrintedStatus = 0xFE;
  TxLogEvent    event             = {};

  while (true) {
    // Sleep with 0% CPU until TaskLoRaTx pushes a TX log event
    if (xQueueReceive(txLogQueue, &event, portMAX_DELAY) == pdTRUE) {
      unsigned long nowMs = millis();

      if (event.droppedSim) {
        if (nowMs - lastPrintMs >= SERIAL_PRINT_MS) {
          lastPrintMs = nowMs;
          Serial.print("[TX][RTOS] Simulated RF dropout window | Drops=");
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
  Serial.println("  ROCKET TRANSMITTER (FreeRTOS) - LoRa 433 MHz      ");
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
  bootTimeMs = millis();

  // Create FreeRTOS synchronization objects BEFORE starting tasks
  sensorMutex = xSemaphoreCreateMutex();
  txLogQueue  = xQueueCreate(LOG_QUEUE_LENGTH, sizeof(TxLogEvent));

  if (sensorMutex == NULL || txLogQueue == NULL) {
    Serial.println("ERROR: Failed to allocate FreeRTOS mutex or queue!");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("LoRa initialized successfully.");
  Serial.print("Radio: 433 MHz | BW 250 kHz | SF 7 | CR 4/5 | Power: ");
  Serial.print(LORA_TX_POWER);
  Serial.println(" dBm");
  Serial.println("Starting FreeRTOS tasks: Sensors (50 Hz) + LoRaTX (10 Hz) + SerialLogger");
  Serial.println();

  // Spawn FreeRTOS tasks (using portable xTaskCreate for ESP32-C3 / ESP32)
  BaseType_t okSensors = xTaskCreate(TaskSensors, "TaskSensors", TASK_STACK_BYTES, NULL, PRIO_SENSOR_TASK, &sensorTaskHandle);
  BaseType_t okLoRaTx  = xTaskCreate(TaskLoRaTx, "TaskLoRaTx", TASK_STACK_BYTES, NULL, PRIO_LORA_TX_TASK, &loraTxTaskHandle);
  BaseType_t okSerial  = xTaskCreate(TaskSerialLogger, "TaskSerial", TASK_STACK_BYTES, NULL, PRIO_SERIAL_TASK, &serialTaskHandle);

  if (okSensors != pdPASS || okLoRaTx != pdPASS || okSerial != pdPASS) {
    Serial.println("ERROR: Failed to create one or more FreeRTOS tasks!");
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  // All active work runs inside TaskSensors, TaskLoRaTx, and TaskSerialLogger.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
