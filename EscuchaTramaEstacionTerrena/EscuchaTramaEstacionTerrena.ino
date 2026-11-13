#include <SPI.h>
#include <LoRa.h>
#include <string.h>

#include "lora_config.h"
#include "telemetry_protocol.h"

/**
 * ============================================================
 * GROUND STATION RECEIVER - ESP32-C3 SUPER MINI
 * Radio: XL1278-SMT / SX1278 (433 MHz, BW 250 kHz, SF7, CR 4/5)
 * ============================================================
 *
 * Output Modes:
 *   - Human-Readable Mode (default NOW): Validates 0xAA sync & CRC-16,
 *     tracks sequence loss / reboots / link health, and prints decoded
 *     telemetry + 1 Hz RF diagnostics to the Arduino Serial Monitor.
 *   - Raw Binary Mode (for LATER with estacion-terrena-pc/main.py):
 *     Suppresses ASCII logs and writes raw 65-byte LoRaPacket frames
 *     directly over USB Serial via Serial.write().
 *
 * Toggle live in Serial Monitor: send 'B' for Binary, 'H' for Human-Readable.
 */

constexpr bool     DEFAULT_HUMAN_READABLE_MODE = true;
constexpr uint32_t STATS_REPORT_INTERVAL_MS    = 1000UL;
constexpr uint32_t LINK_DEGRADED_TIMEOUT_MS    = 1500UL;
constexpr uint32_t LINK_LOST_TIMEOUT_MS        = 3000UL;

bool humanReadableMode = DEFAULT_HUMAN_READABLE_MODE;

// Static receive buffer (avoids dynamic String heap fragmentation)
uint8_t rxBuffer[128];

// Link & integrity counters
uint32_t validPacketsTotal   = 0;
uint32_t lostPacketsTotal    = 0;
uint32_t dupOrOooPackets     = 0;
uint32_t crcErrorsTotal      = 0;
uint32_t lengthSyncErrors    = 0;
uint32_t txRebootCount       = 0;
uint64_t totalBytesReceived  = 0;

// 1-second interval counters
uint32_t intervalPackets     = 0;
uint32_t intervalBytes       = 0;

// Timing & link-state tracking
uint32_t startMs             = 0;
uint32_t lastReportMs        = 0;
uint32_t lastValidPacketMs   = 0;
bool     hasAnyValidPacket   = false;
bool     linkLostAnnounced   = false;
bool     linkDegAnnounced    = false;

// Sequence & reboot tracking
uint16_t lastFrameNumber     = 0;
uint32_t lastPacketTimestamp = 0;

// RF quality metrics
int   lastRssi   = 0;
float lastSnr    = 0.0f;
float avgRssi    = 0.0f;
float avgSnr     = 0.0f;

// Latest decoded frame snapshot
LoRaPacket latestPacket{};

void handleSerialModeCommands() {
    while (Serial.available() > 0) {
        const int ch = Serial.read();
        if (ch == 'b' || ch == 'B') {
            if (humanReadableMode) {
                Serial.println("\n[MODE] Switching to RAW BINARY mode (65B frames for main.py)...");
                Serial.flush();
                humanReadableMode = false;
            }
        } else if (ch == 'h' || ch == 'H') {
            if (!humanReadableMode) {
                humanReadableMode = true;
                Serial.println("\n[MODE] Switched to HUMAN-READABLE debug mode.");
            }
        }
    }
}

void printCompactPacketLine(const LoRaPacket &pkt, int rssi, float snr) {
    Serial.print("[RX] #");
    Serial.print(pkt.frame_number);
    Serial.print(" | ");
    Serial.print(statusToLabel(pkt.status));
    Serial.print(" | Alt=");
    Serial.print(pkt.altura_actual, 1);
    Serial.print("m | P=");
    Serial.print(pkt.pressure, 1);
    Serial.print("hPa | Az=");
    Serial.print(pkt.accel_z, 2);
    Serial.print("g | Sats=");
    Serial.print(pkt.satellites);
    Serial.print(" | RSSI=");
    Serial.print(rssi);
    Serial.print("dBm | SNR=");
    Serial.print(snr, 1);
    Serial.println("dB");
}

void printStatisticsReport(uint32_t nowMs) {
    const float elapsedSec = static_cast<float>(nowMs - lastReportMs) / 1000.0f;
    const float rxRatePps  = (elapsedSec > 0.0f) ? (intervalPackets / elapsedSec) : 0.0f;
    const float rxRateBps  = (elapsedSec > 0.0f) ? (intervalBytes / elapsedSec) : 0.0f;

    const uint32_t expectedTotal = validPacketsTotal + lostPacketsTotal;
    const float lossPercent = (expectedTotal > 0)
        ? (100.0f * static_cast<float>(lostPacketsTotal) / static_cast<float>(expectedTotal))
        : 0.0f;

    Serial.println("==================== GROUND STATION STATS ====================");
    Serial.print("Packets RX: "); Serial.print(validPacketsTotal);
    Serial.print(" | Lost: "); Serial.print(lostPacketsTotal);
    Serial.print(" ("); Serial.print(lossPercent, 2); Serial.print("%)");
    Serial.print(" | Dup/OOO: "); Serial.println(dupOrOooPackets);

    Serial.print("CRC Errors: "); Serial.print(crcErrorsTotal);
    Serial.print(" | Len/Sync Err: "); Serial.print(lengthSyncErrors);
    Serial.print(" | TX Reboots: "); Serial.println(txRebootCount);

    Serial.print("RX Rate: "); Serial.print(rxRatePps, 2); Serial.print(" pkt/s");
    Serial.print(" | Throughput: "); Serial.print(rxRateBps, 1); Serial.print(" B/s");
    Serial.print(" | Total Bytes: "); Serial.println(static_cast<unsigned long>(totalBytesReceived));

    if (hasAnyValidPacket) {
        Serial.print("RSSI: "); Serial.print(lastRssi);
        Serial.print(" dBm (Avg "); Serial.print(avgRssi, 1);
        Serial.print(" dBm) | SNR: "); Serial.print(lastSnr, 1);
        Serial.print(" dB (Avg "); Serial.print(avgSnr, 1); Serial.println(" dB)");

        Serial.print("Latest Telemetry -> T+=");
        Serial.print(latestPacket.timestamp / 1000.0f, 1);
        Serial.print("s | Phase="); Serial.print(statusToLabel(latestPacket.status));
        Serial.print(" | Alt="); Serial.print(latestPacket.altura_actual, 2);
        Serial.print("m | P="); Serial.print(latestPacket.pressure, 2);
        Serial.println("hPa");

        Serial.print("  IMU Accel(g): [");
        Serial.print(latestPacket.accel_x, 2); Serial.print(", ");
        Serial.print(latestPacket.accel_y, 2); Serial.print(", ");
        Serial.print(latestPacket.accel_z, 2); Serial.print("] | Gyro(dps): [");
        Serial.print(latestPacket.gyro_x, 1); Serial.print(", ");
        Serial.print(latestPacket.gyro_y, 1); Serial.print(", ");
        Serial.print(latestPacket.gyro_z, 1); Serial.print("] | T=");
        Serial.print(latestPacket.temp_imu, 1); Serial.println("C");

        Serial.print("  GPS: Lat=");
        Serial.print(latestPacket.latitude, 6);
        Serial.print(" Lon="); Serial.print(latestPacket.longitude, 6);
        Serial.print(" Alt="); Serial.print(latestPacket.altitud_gps, 1);
        Serial.print("m Sats="); Serial.print(latestPacket.satellites);
        Serial.print(" Spd="); Serial.print(latestPacket.ground_speed, 1);
        Serial.print("m/s | Pyro Drogue=");
        Serial.print(latestPacket.pin_frenado ? "ACTIVE" : "OFF");
        Serial.print(" Main=");
        Serial.println(latestPacket.pin_principal ? "ACTIVE" : "OFF");
    } else {
        Serial.println("Status: Waiting for first valid 65-byte LoRaPacket...");
    }

    Serial.print("Uptime: ");
    Serial.print((nowMs - startMs) / 1000.0f, 1);
    Serial.println("s | Mode: HUMAN-READABLE (send 'B' for binary bytes)");
    Serial.println("==============================================================\n");
}

void setup() {
    disableBrownoutDetector();

    Serial.begin(115200);
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

    if (!initLoRaRadio(false)) {
        if (humanReadableMode) {
            uint8_t chipVersion = readSx1278VersionRegister();
            Serial.print("ERROR: Failed to initialize SX1278/XL1278. Reg 0x42 = 0x");
            Serial.println(chipVersion, HEX);
            Serial.println("Check FSPI wiring: SCK=4, MISO=5, MOSI=6, CS=7, RST=10, DIO0=3, VCC=3.3V.");
        }
        while (true) {
            delay(1000);
        }
    }

    startMs      = millis();
    lastReportMs = startMs;

    if (humanReadableMode) {
        Serial.println("LoRa initialized successfully.");
        Serial.println("Freq: 433 MHz | BW: 250 kHz | SF: 7 | CR: 4/5 | Sync: 0x12");
        Serial.println("Expecting 65-byte binary LoRaPacket frames (Sync 0xAA + CRC-16/CCITT-FALSE).");
        Serial.println("Send 'B' over Serial to switch to raw binary mode for main.py.\n");
    }
}

void loop() {
    handleSerialModeCommands();

    const int packetSize = LoRa.parsePacket();
    const uint32_t nowMs = millis();

    if (packetSize > 0) {
        int bytesRead = 0;
        while (LoRa.available() && bytesRead < static_cast<int>(sizeof(rxBuffer))) {
            rxBuffer[bytesRead++] = static_cast<uint8_t>(LoRa.read());
        }
        // Drain any surplus bytes if a malformed oversize packet arrived
        while (LoRa.available()) {
            LoRa.read();
        }

        const int rssi  = LoRa.packetRssi();
        const float snr = LoRa.packetSnr();

        // Raw Binary Bridge mode for estacion-terrena-pc/main.py
        if (!humanReadableMode) {
            if (bytesRead == static_cast<int>(TELEMETRY_FRAME_SIZE) &&
                rxBuffer[0] == TELEMETRY_SYNC_BYTE) {
                Serial.write(rxBuffer, TELEMETRY_FRAME_SIZE);
                validPacketsTotal++;
                totalBytesReceived += TELEMETRY_FRAME_SIZE;
            }
            return;
        }

        // Validate exact 65-byte frame length and 0xAA sync header
        if (bytesRead != static_cast<int>(TELEMETRY_FRAME_SIZE) ||
            rxBuffer[0] != TELEMETRY_SYNC_BYTE) {
            lengthSyncErrors++;
            Serial.print("[WARN] Rejected malformed packet | len=");
            Serial.print(bytesRead);
            Serial.print(" sync=0x");
            Serial.println((bytesRead > 0) ? rxBuffer[0] : 0, HEX);
            return;
        }

        LoRaPacket pkt{};
        memcpy(&pkt, rxBuffer, sizeof(LoRaPacket));

        if (!verifyPacketCrc(pkt)) {
            crcErrorsTotal++;
            Serial.print("[ALERT] CRC-16/CCITT-FALSE mismatch on frame #");
            Serial.print(pkt.frame_number);
            Serial.print(" | rx_crc=0x");
            Serial.println(pkt.crc, HEX);
            return;
        }

        // Valid 65-byte frame confirmed
        if (linkLostAnnounced || linkDegAnnounced) {
            Serial.println("[INFO] RF link recovered: valid telemetry resumed.");
            linkLostAnnounced = false;
            linkDegAnnounced  = false;
        }

        if (hasAnyValidPacket) {
            const bool timestampReset = (pkt.timestamp < lastPacketTimestamp);
            const bool sequenceReset  = (lastFrameNumber > 20U && pkt.frame_number < 10U);

            if (timestampReset && sequenceReset) {
                txRebootCount++;
                Serial.println("[ALERT] In-flight transmitter reboot detected!");
            } else {
                const uint16_t delta = static_cast<uint16_t>(pkt.frame_number - lastFrameNumber);
                if (delta == 0U || delta > 32768U) {
                    dupOrOooPackets++;
                    Serial.print("[WARN] Duplicate or out-of-order frame #");
                    Serial.println(pkt.frame_number);
                } else if (delta > 1U) {
                    const uint16_t gap = delta - 1U;
                    lostPacketsTotal += gap;
                    Serial.print("[WARN] Sequence gap detected: ");
                    Serial.print(gap);
                    Serial.print(" packet(s) lost (");
                    Serial.print(lastFrameNumber);
                    Serial.print(" -> ");
                    Serial.print(pkt.frame_number);
                    Serial.println(")");
                }
            }
        }

        lastFrameNumber     = pkt.frame_number;
        lastPacketTimestamp = pkt.timestamp;
        lastValidPacketMs   = nowMs;
        hasAnyValidPacket   = true;
        latestPacket        = pkt;

        validPacketsTotal++;
        intervalPackets++;
        totalBytesReceived += bytesRead;
        intervalBytes      += bytesRead;

        lastRssi = rssi;
        lastSnr  = snr;
        if (validPacketsTotal == 1U) {
            avgRssi = static_cast<float>(rssi);
            avgSnr  = snr;
        } else {
            avgRssi += (static_cast<float>(rssi) - avgRssi) / static_cast<float>(validPacketsTotal);
            avgSnr  += (snr - avgSnr) / static_cast<float>(validPacketsTotal);
        }

        printCompactPacketLine(pkt, rssi, snr);
    }

    // Link timeout monitoring & 1-second stats reporting (Human-Readable Mode)
    if (humanReadableMode) {
        if (hasAnyValidPacket) {
            const uint32_t silenceMs = nowMs - lastValidPacketMs;
            if (!linkLostAnnounced && silenceMs >= LINK_LOST_TIMEOUT_MS) {
                linkLostAnnounced = true;
                Serial.println("[ALERT] SIGNAL LOST: No valid frames received for > 3.0 s!");
            } else if (!linkDegAnnounced && !linkLostAnnounced && silenceMs >= LINK_DEGRADED_TIMEOUT_MS) {
                linkDegAnnounced = true;
                Serial.println("[WARN] LINK DEGRADED: No valid frames received for > 1.5 s.");
            }
        }

        if (nowMs - lastReportMs >= STATS_REPORT_INTERVAL_MS) {
            printStatisticsReport(nowMs);
            intervalPackets = 0;
            intervalBytes   = 0;
            lastReportMs    = nowMs;
        }
    }
}
