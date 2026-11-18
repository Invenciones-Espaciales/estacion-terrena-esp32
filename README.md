# LoRa 433 MHz Telemetry Firmware — ESP32-C3 Super Mini + XL1278-SMT (SX1278)

Firmware for the **Experimental Rocket V2** Communications & Telemetry subsystem (**Invenciones Espaciales / ENTUSIASMO**), designed to interoperate with [`estacion-terrena-pc`](https://github.com/Invenciones-Espaciales/estacion-terrena-pc) using the **65-byte Little-Endian `LoRaPacket` binary protocol**.

---

## Code Structure (Designed for Quick Editing)

All Arduino sketches are intentionally **self-contained** and organized into **6 numbered sections** so anyone on the team can read a sketch top-to-bottom and modify pins or parameters in seconds:

1. **SECTION 1: USER CONFIGURATION (PINS, RADIO & TIMING)** — All pin assignments, LoRa frequency/bandwidth/SF/CR/TX power, serial baud rate, FreeRTOS rates/priorities, and debug/binary modes in one place at the very top.
2. **SECTION 2: 65-BYTE TELEMETRY PACKET STRUCTURE (`LoRaPacket`)** — Exact packed struct matching `config/frame.yaml`.
3. **SECTION 3: GLOBAL VARIABLES / FREERTOS HANDLES** — Link statistics, mutex/queue handles, and runtime state.
4. **SECTION 4: HELPER FUNCTIONS** — `calculateCRC16`, `getStatusName`, formatting/printing, and packet builder.
5. **SECTION 5: `setup()` / FREERTOS TASKS** — Hardware initialization (`SPI.begin(...)`, `LoRa.setPins(...)`) and FreeRTOS task definitions (`TransmisorCoheteRTOS`).
6. **SECTION 6: `loop()` / `setup() & loop()`** — Main loop or RTOS scheduler startup.

### Files in This Repository

- `TransmisorCohete/TransmisorCohete.ino` — Simple single-loop Rocket TX sketch (10 Hz RF transmission + 1 Hz Serial summary).
- `TransmisorCohete/flight_simulator.h` — 290-second simulated rocket flight curves (`fillSimulatedSensorData`) for bench testing.
- `TransmisorCoheteRTOS/TransmisorCoheteRTOS.ino` — **FreeRTOS OBC (Avionics + COMMS)** sketch exposing `updateRocketTelemetry(const LoRaPacket&)` as a single non-blocking API between the Avionics task (`TaskAvionics`) and the COMMS LoRa tasks (`TaskLoRaTx` + `TaskSerialLogger`).
- `TransmisorCoheteRTOS/flight_simulator.h` — Companion simulator header for `TransmisorCoheteRTOS`.
- `EscuchaTramaEstacionTerrena/EscuchaTramaEstacionTerrena.ino` — Ground Station RX sketch (Human-Readable Serial Monitor cards by default; live toggle to raw 65-byte Binary mode).
- `include/lora_config.h` & `include/telemetry_protocol.h` — Synchronized reference headers used by host verification tests.
- `tests/verify_protocol.cpp` — Host C++ test validating 65-byte layout, offsets, CRC-16/CCITT-FALSE, and simulation timing.

### FreeRTOS Rocket Architecture (`TransmisorCoheteRTOS` — Avionics + COMMS)

Designed as a **Producer-Consumer Mailbox** so the Avionics team never waits on slow `~61.6 ms` LoRa transmissions or USB Serial prints:

- **Single Avionics Entry Point (`updateRocketTelemetry(const LoRaPacket&)`):** Populate your `LoRaPacket` with the 18 sensor/flight fields (`timestamp`, `status`, barometer, IMU, GPS, pyro pins) and pass the struct here (~2 microseconds under `sensorMutex`). `sync`, `frame_number`, and `crc` are computed automatically by COMMS.
- **Single COMMS Initializer (`initCommsLoRa()`):** Initializes SPI/SX1278 pins, creates `sensorMutex` and `txLogQueue`, and launches the background COMMS tasks.
- **`TaskAvionics` (50 Hz / `20 ms`, Priority `3`):** Runs on deterministic `vTaskDelayUntil`, reads sensors/FSM, and calls `updateRocketTelemetry(avionicsData)`.
- **`TaskLoRaTx` (10 Hz / `100 ms`, Priority `2`):** Reads the newest struct from the `sensorMutex` mailbox, stamps `sync = 0xAA` and `frame_number`, calculates `CRC-16/CCITT-FALSE`, transmits the 65-byte frame over SX1278 LoRa, and posts a `TxLogEvent` to `txLogQueue`.
- **`TaskSerialLogger` (Event-driven, Priority `1`):** Sleeps on `txLogQueue` (`portMAX_DELAY`) and prints 1 Hz status / phase-change logs over USB Serial.

---

## Hardware Wiring (ESP32-C3 Super Mini <-> XL1278-SMT)

Uses the original safe FSPI hardware pins (`4..7`) and safe control GPIOs (`3`, `10`), avoiding ESP32-C3 internal SPI flash pins (`GPIOs 12..17`):

| Constant Name | ESP32-C3 GPIO | SX1278 / XL1278 Pin | Function |
| :--- | :--- | :--- | :--- |
| `PIN_SCK` | `GPIO 4` | `SCK` | SPI Clock |
| `PIN_MISO` | `GPIO 5` | `MISO` | SPI MISO |
| `PIN_MOSI` | `GPIO 6` | `MOSI` | SPI MOSI |
| `PIN_CS` | `GPIO 7` | `NSS` | SPI Chip Select |
| `PIN_RST` | `GPIO 10` | `RESET` | Hardware Reset |
| `PIN_DIO0` | `GPIO 3` | `DIO0` | Packet Ready / TX Done |
| — | `3V3` / `GND` | `VCC` / `GND` | 3.3 V Power Supply |

> **Power Safety Note:** `LORA_TX_POWER` defaults to `2` dBm so bench testing over USB does not trigger 3.3 V brownout resets. Before raising `LORA_TX_POWER` to `17`–`20` dBm for field range testing, wire a `220–470 uF` decoupling capacitor and a dedicated `3.3V >= 500 mA` regulator on the SX1278 VCC rail.

---

## Switching Ground Station Mode (`'H'` Human-Readable <-> `'B'` 97-Byte Binary)

1. **Human-Readable Debug (`'H'`, Default on Boot):** `HUMAN_READABLE_DEFAULT = true` prints decoded telemetry blocks and 1 Hz `========== STATISTICS ==========` reports at `115200` baud.
2. **Dashboard Binary Mode (`'B'`, Watchdog-Protected):** When `estacion-terrena-pc/main.py` connects, it automatically sends `'B'` (plus a `2.0 s` keepalive) and returns `'H'` on clean exit. The Ground Station ESP32 emits **97-byte USB frames** (`32B GroundStationHeader` starting with `0xAB` + `65B` untouched `LoRaPacket` starting with `0xAA`), delivering live `rssi`, `snr`, and RX counters (`rx_packets`, `rx_lost`, `rx_crc_errors`, `rx_size_errors`, `tx_reboots`, `gs_uptime_ms`, `gs_flags`). If the dashboard crashes or disconnects for `> 6.0 s` (`BINARY_WATCHDOG_MS`), the ESP32 automatically reverts to `'H'` mode.
3. **Manual Sticky Binary (`'M'`):** Enters 97-byte Binary mode without the 6 s keepalive timeout until `'H'` is sent.

---

## Host Protocol Verification

```bash
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude -ITransmisorCohete tests/verify_protocol.cpp -o /tmp/verify_protocol && /tmp/verify_protocol
```
