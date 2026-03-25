/*
 * SN2-Particle-SDI12-Terminal
 * SDI-12 Lab Terminal Tool for Particle Boron
 *
 * Purpose:
 *   A USB serial terminal that lets you interact with SDI-12 sensors directly.
 *   Intended for bench/lab use — wire up sensors, open a serial monitor at
 *   115200 baud, and use the built-in commands to scan, read, and reconfigure sensors.
 *
 * Built-in commands (type into serial monitor and press Enter):
 *   SCAN          — Poll addresses 0-9, list all responding sensors with their info
 *   READ <addr>   — Run a full M!/D0! measurement cycle on the given address
 *   ADDR <old> <new> — Change a sensor's SDI-12 address (sends aAb! command)
 *   INFO <addr>   — Send the identify command (aI!) and print the full sensor info
 *   HELP          — Print this command list
 *   <any>         — Any other input is sent directly as a raw SDI-12 command
 *                   (e.g. "0M!" or "0I!" or "?!")
 *
 * Pin config (same as SN2-Particle-SDI12 field firmware):
 *   D4 — SDI-12 data bus
 *   D2 — Sensor power (set HIGH on boot, stays on during terminal session)
 *
 * Written by: Nick Perlaky
 * Based on SDI12_Direct_Terminal_AT (Asset Tracker version) and SN2-Particle-SDI12
 */

#include "Particle.h"
#include <SDI12.h>

SYSTEM_MODE(MANUAL);   // No cloud connection needed — USB serial only
SYSTEM_THREAD(ENABLED);

// ==================== PIN CONFIG ====================
constexpr pin_t SDI12_DATA_PIN   = D4;
constexpr pin_t SENSOR_POWER_PIN = D2;
constexpr pin_t STATUS_LED_PIN   = D7;

// ==================== CONFIG ====================
constexpr uint32_t SERIAL_BAUD          = 115200;
constexpr uint32_t SDI12_CMD_TIMEOUT_MS = 500;   // Wait for sensor response (ms)
constexpr uint32_t SDI12_MEAS_EXTRA_MS  = 1000;  // Extra wait added to M! ttt value
constexpr uint32_t SERIAL_WAIT_MS       = 10000; // Wait up to 10 s for USB serial

// ==================== GLOBALS ====================
SDI12 mySDI12(SDI12_DATA_PIN);

String inputBuffer = "";
bool   inputReady  = false;

// ==================== FUNCTION DECLARATIONS ====================
void cmdScan();
void cmdRead(const String &addr);
void cmdChangeAddress(const String &oldAddr, const String &newAddr);
void cmdInfo(const String &addr);
void cmdHelp();
void sendRawCommand(const String &command);

bool     checkSDI12Active(char address);
bool     readSDI12Sensor(char address, String &data);
String   sdi12Info(char address);
String   readSDI12Response(uint32_t timeoutMs = SDI12_CMD_TIMEOUT_MS);
void     processCommand(const String &line);
char     decToChar(byte i);

// ==================== SETUP ====================
void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH); // On solid while terminal is running

  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, HIGH); // Power sensors immediately

  Serial.begin(SERIAL_BAUD);

  // Wait for USB serial to connect (important on Boron)
  uint32_t serialStart = millis();
  while (!Serial.isConnected() && (millis() - serialStart) < SERIAL_WAIT_MS) {
    delay(100);
  }

  delay(500); // Let sensors stabilize after power-on

  // Initialize SDI-12 bus
  mySDI12.begin();
  delay(100);

  Serial.println();
  Serial.println("========================================");
  Serial.println("  SN2-Particle-SDI12-Terminal");
  Serial.println("  Particle Boron SDI-12 Lab Tool");
  Serial.println("========================================");
  Serial.println();
  Serial.printlnf("SDI-12 data pin : D%d", (int)SDI12_DATA_PIN);
  Serial.printlnf("Sensor power pin: D%d  [currently HIGH]", (int)SENSOR_POWER_PIN);
  Serial.println();
  cmdHelp();
  Serial.println();
  Serial.println("Ready. Type a command or raw SDI-12 command:");
  Serial.print("> ");
}

// ==================== LOOP ====================
void loop() {
  // Accumulate characters from USB serial into inputBuffer
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\r') {
      // Ignore bare carriage return — we trigger on '\n'
      continue;
    }

    if (c == '\n') {
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
        Serial.println(); // Echo newline
        processCommand(inputBuffer);
        inputBuffer = "";
        Serial.println();
        Serial.print("> ");
      }
    } else {
      Serial.print(c); // Echo character back
      inputBuffer += c;
    }
  }
}

// ==================== COMMAND DISPATCHER ====================
void processCommand(const String &line) {
  // Make a working copy and normalize case for the command keyword only
  String trimmed = line;
  trimmed.trim();

  // Extract first token (the command word)
  int spaceIdx = trimmed.indexOf(' ');
  String cmd   = (spaceIdx > 0) ? trimmed.substring(0, spaceIdx) : trimmed;
  String rest  = (spaceIdx > 0) ? trimmed.substring(spaceIdx + 1) : "";
  rest.trim();

  String cmdUpper = cmd;
  cmdUpper.toUpperCase();

  if (cmdUpper == "SCAN") {
    cmdScan();

  } else if (cmdUpper == "READ") {
    if (rest.length() == 0) {
      Serial.println("Usage: READ <address>   e.g.  READ 0");
    } else {
      cmdRead(rest);
    }

  } else if (cmdUpper == "ADDR") {
    // Expect two tokens: old address, new address
    int sp2 = rest.indexOf(' ');
    if (sp2 < 0 || rest.length() < 3) {
      Serial.println("Usage: ADDR <old_address> <new_address>   e.g.  ADDR 0 3");
    } else {
      String oldA = rest.substring(0, sp2);
      String newA = rest.substring(sp2 + 1);
      oldA.trim();
      newA.trim();
      cmdChangeAddress(oldA, newA);
    }

  } else if (cmdUpper == "INFO") {
    if (rest.length() == 0) {
      Serial.println("Usage: INFO <address>   e.g.  INFO 0");
    } else {
      cmdInfo(rest);
    }

  } else if (cmdUpper == "HELP") {
    cmdHelp();

  } else {
    // Raw SDI-12 command passthrough
    sendRawCommand(trimmed);
  }
}

// ==================== COMMANDS ====================

/**
 * Scan all 10 addresses ('0'-'9'), print a table of responding sensors.
 */
void cmdScan() {
  Serial.println("Scanning SDI-12 addresses 0-9...");
  Serial.println("----------------------------------------");

  int found = 0;
  for (byte i = 0; i < 10; i++) {
    char addr = decToChar(i);
    Serial.printf("  Address '%c' ... ", addr);

    // Send acknowledge command
    String ackCmd = String(addr) + "!";
    mySDI12.clearBuffer();
    mySDI12.sendCommand(ackCmd.c_str());

    String ackResp = readSDI12Response(SDI12_CMD_TIMEOUT_MS);

    if (ackResp.length() > 0) {
      // Got a response — get full info
      String info = sdi12Info(addr);
      if (info.length() >= 20) {
        found++;
        Serial.println("FOUND");
        // Parse info string: addr(1) + version(2) + vendor(8) + model(6) + firmware(3) + serial(rest)
        if (info.length() > 3) {
          Serial.printf("    Version:  %.1f\n",  info.substring(1, 3).toFloat() / 10.0f);
        }
        if (info.length() > 11) {
          Serial.printf("    Vendor:   %s\n",   info.substring(3, 11).c_str());
        }
        if (info.length() > 17) {
          Serial.printf("    Model:    %s\n",   info.substring(11, 17).c_str());
        }
        if (info.length() > 20) {
          Serial.printf("    Firmware: %s\n",   info.substring(17, 20).c_str());
        }
        if (info.length() > 20) {
          Serial.printf("    Serial:   %s\n",   info.substring(20).c_str());
        }
      } else {
        // Responded to ack but gave short info — still present
        found++;
        Serial.println("FOUND (short info response)");
        Serial.printf("    Raw response: '%s'\n", ackResp.c_str());
      }
    } else {
      Serial.println("no response");
    }

    delay(100);
  }

  Serial.println("----------------------------------------");
  Serial.printf("Scan complete: %d sensor(s) found\n", found);
}

/**
 * Run a full M!/D0! measurement cycle on one sensor and print the result.
 */
void cmdRead(const String &addrStr) {
  if (addrStr.length() == 0) return;
  char addr = addrStr.charAt(0);

  Serial.printf("Reading sensor at address '%c'...\n", addr);

  String data;
  // Send M! command
  String mCmd = String(addr) + "M!";
  mySDI12.clearBuffer();
  mySDI12.sendCommand(mCmd.c_str());

  String mResp = readSDI12Response(SDI12_CMD_TIMEOUT_MS);

  if (mResp.length() < 4) {
    Serial.printf("  ERROR: No/invalid M! response: '%s'\n", mResp.c_str());
    return;
  }

  int waitSec  = mResp.substring(1, 4).toInt();
  int numVals  = mResp.substring(4).toInt();
  Serial.printf("  M! response: '%s'  (wait %ds, %d value(s))\n",
                mResp.c_str(), waitSec, numVals);

  if (numVals == 0) {
    Serial.println("  Sensor reports 0 values — nothing to read");
    return;
  }

  // Wait for measurement
  uint32_t waitMs = (uint32_t)(waitSec * 1000) + SDI12_MEAS_EXTRA_MS;
  Serial.printf("  Waiting %ums for measurement...\n", (unsigned)waitMs);
  uint32_t waitStart = millis();
  while ((millis() - waitStart) < waitMs) {
    if (mySDI12.available()) mySDI12.clearBuffer(); // Discard service requests
    delay(10);
  }

  // Send D0!
  String dCmd = String(addr) + "D0!";
  mySDI12.clearBuffer();
  mySDI12.sendCommand(dCmd.c_str());
  String dResp = readSDI12Response(SDI12_CMD_TIMEOUT_MS);

  if (dResp.length() > 1) {
    Serial.printf("  Data (raw D0! response): '%s'\n", dResp.c_str());
    Serial.printf("  Values: '%s'\n", dResp.substring(1).c_str()); // Strip address prefix
  } else {
    Serial.printf("  ERROR: No D0! response: '%s'\n", dResp.c_str());
  }
}

/**
 * Change the SDI-12 address of a sensor using the aAb! command.
 * Both old and new address must be single characters '0'-'9'.
 */
void cmdChangeAddress(const String &oldAddrStr, const String &newAddrStr) {
  if (oldAddrStr.length() == 0 || newAddrStr.length() == 0) return;

  char oldAddr = oldAddrStr.charAt(0);
  char newAddr = newAddrStr.charAt(0);

  // Validate new address is 0-9
  if (newAddr < '0' || newAddr > '9') {
    Serial.printf("  ERROR: New address '%c' is not in range 0-9\n", newAddr);
    Serial.println("  (This firmware only supports numeric addresses 0-9)");
    return;
  }

  Serial.printf("Changing address '%c' -> '%c'...\n", oldAddr, newAddr);

  // Build aAb! command
  String addrCmd = String(oldAddr) + "A" + String(newAddr) + "!";
  Serial.printf("  Sending: '%s'\n", addrCmd.c_str());

  mySDI12.clearBuffer();
  mySDI12.sendCommand(addrCmd.c_str());

  String resp = readSDI12Response(SDI12_CMD_TIMEOUT_MS);

  if (resp.length() > 0) {
    Serial.printf("  Response: '%s'\n", resp.c_str());
    // A successful address change echoes the new address
    if (resp.indexOf(newAddr) >= 0) {
      Serial.printf("  SUCCESS: Sensor now at address '%c'\n", newAddr);
    } else {
      Serial.println("  WARNING: Unexpected response — verify address manually with SCAN");
    }
  } else {
    Serial.println("  ERROR: No response from sensor. Check address and connection.");
  }
}

/**
 * Send the identify command (aI!) and print full sensor info.
 */
void cmdInfo(const String &addrStr) {
  if (addrStr.length() == 0) return;
  char addr = addrStr.charAt(0);

  Serial.printf("Sensor info for address '%c':\n", addr);

  String info = sdi12Info(addr);
  if (info.length() == 0) {
    Serial.println("  No response");
    return;
  }

  Serial.printf("  Raw I! response: '%s'\n", info.c_str());
  if (info.length() >= 3) Serial.printf("  Version:  %.1f\n",  info.substring(1, 3).toFloat() / 10.0f);
  if (info.length() >= 11) Serial.printf("  Vendor:   %s\n",  info.substring(3, 11).c_str());
  if (info.length() >= 17) Serial.printf("  Model:    %s\n",  info.substring(11, 17).c_str());
  if (info.length() >= 20) Serial.printf("  Firmware: %s\n",  info.substring(17, 20).c_str());
  if (info.length() > 20)  Serial.printf("  Serial:   %s\n",  info.substring(20).c_str());
}

/**
 * Print the help text.
 */
void cmdHelp() {
  Serial.println("Commands:");
  Serial.println("  SCAN               Scan addresses 0-9, list found sensors");
  Serial.println("  READ <addr>        Full M!/D0! measurement  (e.g. READ 0)");
  Serial.println("  INFO <addr>        Sensor identification    (e.g. INFO 0)");
  Serial.println("  ADDR <old> <new>   Change sensor address    (e.g. ADDR 0 3)");
  Serial.println("  HELP               Show this help text");
  Serial.println("  <raw command>      Send anything else directly to the bus");
  Serial.println("                       e.g.  0M!   0I!   ?!   0D0!");
}

/**
 * Send a raw SDI-12 command and print the response.
 * Suitable for arbitrary commands the user types that don't match built-ins.
 */
void sendRawCommand(const String &command) {
  Serial.printf("Sending raw command: '%s'\n", command.c_str());
  mySDI12.clearBuffer();
  mySDI12.sendCommand(command.c_str());

  String resp = readSDI12Response(SDI12_CMD_TIMEOUT_MS);

  if (resp.length() > 0) {
    Serial.printf("Response: '%s'\n", resp.c_str());
  } else {
    Serial.println("(no response)");
  }
}

// ==================== SDI-12 HELPERS ====================

/**
 * Read all available SDI-12 response characters into a string, with timeout.
 * Strips CR and LF.
 */
String readSDI12Response(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (!mySDI12.available() && (millis() - start) < timeoutMs);

  String resp = "";
  uint32_t lastCharTime = millis();
  // SDI-12 is 1200 baud, meaning each character takes ~8.33ms to transmit.
  // Wait up to 30ms after the last character to see if more arrive.
  while ((millis() - lastCharTime) < 30) {
    if (mySDI12.available()) {
      char c = mySDI12.read();
      lastCharTime = millis();
      if (c == '\n') break; // End of response
      if (c != '\r') resp += c;
    }
    delay(1);
  }
  
  resp.trim();
  mySDI12.clearBuffer();
  return resp;
}

/**
 * Check if a sensor is active at the given address (ack + info challenge).
 * Returns true if a sensor responds with a valid I! response (>= 20 chars).
 */
bool checkSDI12Active(char address) {
  String ackCmd = String(address) + "!";
  for (int attempt = 0; attempt < 3; attempt++) {
    mySDI12.clearBuffer();
    mySDI12.sendCommand(ackCmd.c_str());
    String ack = readSDI12Response(SDI12_CMD_TIMEOUT_MS);
    if (ack.length() > 0) {
      String info = sdi12Info(address);
      if (info.length() >= 20) return true;
    }
  }
  return false;
}

/**
 * Send aI! and return the response string (or empty on failure).
 */
String sdi12Info(char address) {
  String infoCmd = String(address) + "I!";
  mySDI12.clearBuffer();
  mySDI12.sendCommand(infoCmd.c_str());
  return readSDI12Response(SDI12_CMD_TIMEOUT_MS);
}

/**
 * Convert decimal 0-9 to SDI-12 address character '0'-'9'.
 */
char decToChar(byte i) {
  if (i < 10) return i + '0';
  return '?';
}
