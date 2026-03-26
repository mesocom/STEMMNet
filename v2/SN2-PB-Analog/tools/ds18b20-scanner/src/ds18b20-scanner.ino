#include "Particle.h"

#define ONEWIRE_USE_FAST_PINIO 0
#ifndef HAL_Pin_Mode
inline void HAL_Pin_Mode(pin_t pin, PinMode mode) { pinMode(pin, mode); }
#endif

#include "OneWire.h"
#include "DS18.h"

SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

constexpr pin_t SENSOR_POWER_PIN = D2;
constexpr pin_t ONEWIRE_PIN = D4;
constexpr unsigned long SCAN_INTERVAL_MS = 5000;

DS18 ds18(ONEWIRE_PIN);
String scannedAddys = ""; 

const char *ds18TypeToString(int typeInt) {
  DS18Type type = (DS18Type)typeInt;
  switch (type) {
    case WIRE_DS18B20:
      return "DS18B20";
    case WIRE_DS1820:
      return "DS1820";
    case WIRE_DS1822:
      return "DS1822";
    case WIRE_DS2438:
      return "DS2438";
    default:
      return "Unknown";
  }
}

void printAddress(const uint8_t addr[8]) {
  for (size_t i = 0; i < 8; i++) {
    if (addr[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(addr[i], HEX);
    if (i < 7) {
      Serial.print(':');
    }
  }
}

void scanBus() {
  Serial.println("Scanning DS18 bus...");
  scannedAddys = ""; // Clear previous results

  uint8_t addr[8];
  uint8_t scratch[9];
  unsigned int found = 0;

  while (true) {
    bool readOk = ds18.read();

    if (!readOk) {
      if (ds18.searchDone()) {
        break;
      }
      if (ds18.crcError()) {
        Serial.println("  Skipped device due to CRC error");
        continue;
      }
      Serial.println("  Skipped unsupported device");
      continue;
    }

    ds18.addr(addr);
    ds18.data(scratch);

    // Build the hex string for remote retrieval
    char hexAddr[17];
    snprintf(hexAddr, sizeof(hexAddr), "%02X%02X%02X%02X%02X%02X%02X%02X",
             addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7]);
    
    if (scannedAddys.length() > 0) scannedAddys += ",";
    scannedAddys += hexAddr;

    Serial.print("  Sensor ");
    Serial.print(found);
    Serial.print(" | ROM=");
    printAddress(addr);
    Serial.print(" | Type=");
    Serial.print(ds18TypeToString(ds18.type()));
    Serial.print(" | TempC=");
    Serial.println(ds18.celsius(), 4);

    found++;
  }

  if (found == 0) {
    Serial.println("  No DS18 sensors found");
    scannedAddys = "none";
  }
  Serial.println();
}

void setup() {
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, HIGH);

  Serial.begin(115200);
  waitFor(Serial.isConnected, 10000);

  Serial.println("DS18B20 discovery (Boron)");
  Serial.println("Power pin: D2, Data pin: D4");
  Serial.println("===========================");

  Particle.variable("scannedAddys", scannedAddys);
}

void loop() {
  scanBus();
  delay(SCAN_INTERVAL_MS);
}