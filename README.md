# LoRa 433 MHz Telemetry Firmware — ESP32-C3 Super Mini + XL1278-SMT (SX1278)

Firmware for the **Experimental Rocket V2** Communications & Telemetry subsystem (**Invenciones Espaciales / ENTUSIASMO**), designed to interoperate with [`estacion-terrena-pc`](https://github.com/Invenciones-Espaciales/estacion-terrena-pc) using the **65-byte Little-Endian `LoRaPacket` binary protocol**.

---

## Code Structure (Designed for Quick Editing)

Both Arduino sketches are intentionally **self-contained** and organized into **6 numbered sections** so anyone on the team can read a sketch top-to-bottom and modify pins or parameters in seconds:

1. **SECTION 1: USER CONFIGURATION (PINS, RADIO & TIMING)** — All pin assignments, LoRa frequency/bandwidth/SF/CR/TX power, serial baud rate, intervals, and debug/binary modes in one place at the very top.
2. **SECTION 2: 65-BYTE TELEMETRY PACKET STRUCTURE (`LoRaPacket`)** — Exact packed struct matching `config/frame.yaml`.
3. **SECTION 3: GLOBAL VARIABLES** — Link statistics and runtime state.
4. **SECTION 4: HELPER FUNCTIONS** — `calculateCRC16`, `getStatusName`, formatting/printing, and packet builder.
5. **SECTION 5: `setup()`** — Initializes Serial, `SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS)`, and `LoRa.setPins(PIN_CS, PIN_RST, PIN_DIO0)`.
6. **SECTION 6: `loop()`** — Runs the 10 Hz TX simulation loop or RX packet listener.

### Files in This Repository

- `TransmisorCohete/TransmisorCohete.ino` — Rocket TX sketch (10 Hz RF transmission + 1 Hz Serial status summary).
- `TransmisorCohete/flight_simulator.h` — 290-second simulated rocket flight curves (`fillSimulatedSensorData`) for bench testing.
- `EscuchaTramaEstacionTerrena/EscuchaTramaEstacionTerrena.ino` — Ground Station RX sketch (Human-Readable Serial Monitor cards by default; live toggle to raw 65-byte Binary mode).
- `include/lora_config.h` & `include/telemetry_protocol.h` — Synchronized reference headers used by host verification tests.
- `tests/verify_protocol.cpp` — Host C++ test validating 65-byte layout, offsets, CRC-16/CCITT-FALSE, and simulation timing.

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

## Switching Ground Station Mode (Human-Readable <-> PC Binary)

1. **Human-Readable Debug (Default):** `HUMAN_READABLE_MODE = true` prints decoded telemetry blocks and 1 Hz `========== STATISTICS ==========` reports at `115200` baud.
2. **Raw 65-Byte Binary Mode (`estacion-terrena-pc/main.py`):** Send `B` over Serial (or set `HUMAN_READABLE_MODE = false` in Section 1) to forward raw 65-byte `LoRaPacket` frames via `Serial.write`. Send `H` to return to Human-Readable mode.

---

## Host Protocol Verification

```bash
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude -ITransmisorCohete tests/verify_protocol.cpp -o /tmp/verify_protocol && /tmp/verify_protocol
```
