/*
 * ds18b20-console-scan
 * STEMMNet SN2-PB-Analog — DS18B20 Address Scanner
 *
 * Scans the OneWire bus on D4 (sensor power on D2) and prints every
 * discovered sensor's ROM address, type, and temperature to the USB
 * serial console.  No cloud connection or Particle variable publishing.
 *
 * Pin mapping matches sn2-pb-analog.ino:
 *   SENSOR_POWER_PIN = D2
 *   ONEWIRE_PIN      = D4
 *
 * Usage:
 *   1. Flash to Boron via Particle CLI or Workbench.
 *   2. Open serial monitor at 115200 baud.
 *   3. The bus is scanned once at startup, then every SCAN_INTERVAL_MS.
 *   4. Copy the "Array initialiser" lines into dsAddresses[] in the main firmware.
 */

#include "Particle.h"

#define ONEWIRE_USE_FAST_PINIO 0
#ifndef HAL_Pin_Mode
inline void HAL_Pin_Mode(pin_t pin, PinMode mode) { pinMode(pin, mode); }
#endif

#include "DS18.h"

// Run without cloud — keeps startup fast and avoids waiting for cellular
SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

// ==================== PIN CONFIG ====================
// Must match sn2-pb-analog.ino
constexpr pin_t SENSOR_POWER_PIN = D2;
constexpr pin_t ONEWIRE_PIN      = D4;

constexpr unsigned long SCAN_INTERVAL_MS = 5000;

DS18 ds18(ONEWIRE_PIN);

// ---------- helpers ----------

const char *typeLabel(int typeInt) {
  switch ((DS18Type)typeInt) {
    case WIRE_DS18B20: return "DS18B20";
    case WIRE_DS1820:  return "DS1820";
    case WIRE_DS1822:  return "DS1822";
    case WIRE_DS2438:  return "DS2438";
    default:           return "Unknown";
  }
}

/** Print an 8-byte ROM address as colon-separated hex, e.g. 28:BA:81:1F:0F:00:00:2D */
void printROM(const uint8_t addr[8]) {
  for (size_t i = 0; i < 8; i++) {
    if (addr[i] < 0x10) Serial.print('0');
    Serial.print(addr[i], HEX);
    if (i < 7) Serial.print(':');
  }
}

/** Print ROM as a C-array initialiser row ready to paste into dsAddresses[] */
void printArrayInit(const uint8_t addr[8]) {
  Serial.print("  {");
  for (size_t i = 0; i < 8; i++) {
    Serial.print("0x");
    if (addr[i] < 0x10) Serial.print('0');
    Serial.print(addr[i], HEX);
    if (i < 7) Serial.print(", ");
  }
  Serial.print("}");
}

// ---------- scan ----------

void scanBus() {
  Serial.println("┌─ DS18 Bus Scan ───────────────────────────────────┐");

  uint8_t addr[8];
  unsigned int found = 0;

  while (true) {
    bool ok = ds18.read();

    if (!ok) {
      if (ds18.searchDone()) break;
      if (ds18.crcError()) {
        Serial.println("│  [SKIP] CRC error — check wiring / pull-up resistor");
        continue;
      }
      Serial.println("│  [SKIP] Unsupported device");
      continue;
    }

    ds18.addr(addr);

    Serial.print("│  Sensor ");
    Serial.print(found);
    Serial.print("  ROM=");
    printROM(addr);
    Serial.print("  Type=");
    Serial.print(typeLabel(ds18.type()));
    Serial.print("  Temp=");
    Serial.print(ds18.celsius(), 2);
    Serial.println(" C");

    Serial.print("│           Array initialiser: ");
    printArrayInit(addr);
    Serial.println(",");

    found++;
  }

  if (found == 0) {
    Serial.println("│  No sensors found. Check power (D2 HIGH), pull-up, and wiring.");
  }

  Serial.println("└────────────────────────────────────────────────────┘");
  Serial.printlnf("  Total: %u sensor(s) detected", found);
  Serial.println();
}

// ==================== SETUP / LOOP ====================

void setup() {
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, HIGH);   // Power up sensor rail
  pinMode(ONEWIRE_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  waitFor(Serial.isConnected, 10000);

  Serial.println();
  Serial.println("=========================================");
  Serial.println("  DS18B20 Console Scanner");
  Serial.println("  SN2-PB-Analog  |  Particle Boron");
  Serial.println("  Power pin: D2  |  Data pin: D4");
  Serial.println("=========================================");
  Serial.println();

  scanBus();  // Immediate first scan on boot
}

void loop() {
  delay(SCAN_INTERVAL_MS);
  scanBus();
}
