#include <SPI.h>
#include <LoRa.h>

#include "lora_config.h"
#include "telemetry_protocol.h"
#include "flight_simulator.h"

/**
 * ============================================================
 * ROCKET TRANSMITTER (OBC SIMULATOR) - ESP32-C3 SUPER MINI
 * Radio: XL1278-SMT / SX1278 (433 MHz, BW 250 kHz, SF7, CR 4/5)
 * ============================================================
 *
 * Transmits the definitive 65-byte Little-Endian binary LoRaPacket
 * over RF at 10 Hz (100 ms interval), and logs a 1 Hz human-readable
 * mission status summary over USB Serial for bench verification.
 */

constexpr uint32_t SERIAL_LOG_INTERVAL_MS = 1000UL;

FlightSimulator simulator;
uint32_t lastTxMs       = 0;
uint32_t lastSerialMs   = 0;
uint8_t  lastStatusCode = 0xFF;
uint32_t txSuccessCount = 0;
uint32_t txDropSimCount = 0;
uint32_t txErrorCount   = 0;

void printHumanReadableTxStatus(const LoRaPacket &pkt, uint32_t txDurationMs, bool sentOk) {
    Serial.print("[TX] Frame=");
    Serial.print(pkt.frame_number);
    Serial.print(" | T+=");
    Serial.print(pkt.timestamp / 1000.0f, 1);
    Serial.print("s | Phase=");
    Serial.print(statusToLabel(pkt.status));
    Serial.print(" | Alt=");
    Serial.print(pkt.altura_actual, 1);
    Serial.print("m | Az=");
    Serial.print(pkt.accel_z, 2);
    Serial.print("g | ToA=");
    Serial.print(txDurationMs);
    Serial.print("ms | CRC=0x");
    Serial.print(pkt.crc, HEX);
    Serial.print(" | Status=");
    Serial.println(sentOk ? "OK" : "RF_ERR");
}

void setup() {
    disableBrownoutDetector();

    Serial.begin(115200);
    while (!Serial && millis() < 2500) {
        delay(10);
    }
    delay(300);

    Serial.println();
    Serial.println("====================================================");
    Serial.println("   ROCKET TRANSMITTER - LoRa 433 MHz (ESP32-C3)     ");
    Serial.println("====================================================");

    if (!initLoRaRadio(true)) {
        uint8_t chipVersion = readSx1278VersionRegister();
        Serial.print("ERROR: Failed to initialize SX1278/XL1278. Reg 0x42 = 0x");
        Serial.println(chipVersion, HEX);
        Serial.println("Check FSPI wiring: SCK=4, MISO=5, MOSI=6, CS=7, RST=10, DIO0=3, VCC=3.3V.");
        while (true) {
            delay(1000);
        }
    }

    srand(static_cast<unsigned int>(micros()));
    const uint32_t now = millis();
    simulator.reset(now);
    lastTxMs     = now;
    lastSerialMs = now;

    Serial.println("LoRa initialized successfully.");
    Serial.print("Freq: 433 MHz | BW: 250 kHz | SF: 7 | CR: 4/5 | Power: ");
    Serial.print(LORA_TX_POWER_DBM);
    Serial.println(" dBm");
    Serial.print("Frame size: ");
    Serial.print(sizeof(LoRaPacket));
    Serial.println(" bytes | Cadence: 10 Hz (100 ms)");
    Serial.println("Starting 290-second flight simulation...");
    Serial.println();
}

void loop() {
    const uint32_t now = millis();

    if (now - lastTxMs >= TELEMETRY_INTERVAL_MS) {
        lastTxMs += TELEMETRY_INTERVAL_MS;
        // Prevent burst catch-up if a single iteration overruns
        if (now - lastTxMs >= TELEMETRY_INTERVAL_MS) {
            lastTxMs = now;
        }

        LoRaPacket packet{};
        if (!simulator.buildNextPacket(now, packet)) {
            txDropSimCount++;
            if (now - lastSerialMs >= SERIAL_LOG_INTERVAL_MS) {
                lastSerialMs = now;
                Serial.print("[TX] Simulated RF dropout window | Drops=");
                Serial.println(txDropSimCount);
            }
            return;
        }

        const uint32_t txStart = millis();
        LoRa.beginPacket();
        LoRa.write(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        const int result = LoRa.endPacket();
        const uint32_t txDuration = millis() - txStart;

        const bool sentOk = (result == 1);
        if (sentOk) {
            txSuccessCount++;
        } else {
            txErrorCount++;
        }

        const bool phaseChanged = (packet.status != lastStatusCode);
        if (phaseChanged || (now - lastSerialMs >= SERIAL_LOG_INTERVAL_MS)) {
            lastSerialMs   = now;
            lastStatusCode = packet.status;
            printHumanReadableTxStatus(packet, txDuration, sentOk);
        }
    }
}
