/*
 * STEMMNet Station v2 — Particle Boron — Analog/Temperature Sensor Array
 */

#include "Particle.h"

#define ONEWIRE_USE_FAST_PINIO 0
#ifndef HAL_Pin_Mode
inline void HAL_Pin_Mode(pin_t pin, PinMode mode) { pinMode(pin, mode); }
#endif

#define ARDUINOJSON_ENABLE_PROGMEM 0
#include <ArduinoJson.h>
#include <SdFat.h>
#include <Adafruit_ADS1X15.h>
#include "DS18.h"

SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);
#include <chrono>

// ==================== PIN CONFIG ====================
constexpr pin_t MICROSD_CS = SS; // D14 / A5
constexpr pin_t SENSOR_POWER_PIN = D2;
constexpr pin_t STATUS_LED_PIN = D7;
constexpr pin_t ONEWIRE_PIN = D4;
constexpr pin_t SD_CARD_POWER_PIN = PIN_INVALID; // update if carrier exposes dedicated power enable

// ==================== CONFIGURATION ====================
constexpr bool SERIAL_DEBUG = true;

// Station identification
const char AUTH_TOKEN[] = ""; // Update with your station token

constexpr size_t POINT_LENGTH_BYTES = 1024;
constexpr size_t MAX_QUEUE_LINE_BYTES = 4096;
constexpr unsigned long FIRST_BOOT_CONNECT_TIMEOUT_MS = 5UL * 60UL * 1000UL;
constexpr unsigned long STANDARD_CONNECT_TIMEOUT_MS = 60UL * 1000UL;
constexpr unsigned long HTTP_RESPONSE_TIMEOUT_MS = 15000UL;

const char INGEST_HOST[] = "api.alclimate.com";
constexpr int INGEST_PORT = 80;
const char INGEST_PATH[] = "/ingest/auburnh2onet.php";
const char CONFIG_PATH[] = "/ingest/config.php";
constexpr int MIN_DATA_INTERVAL = 60;
constexpr int MAX_DATA_INTERVAL = 86400;

constexpr float ADS1115_MV_PER_BIT_GAIN_ONE = 0.125f;

constexpr int MAX_DS18_SENSORS = 5;
uint8_t dsAddresses[MAX_DS18_SENSORS][8] = {
  {0x28, 0xBA, 0x81, 0x1F, 0x0F, 0x00, 0x00, 0x2D},
  {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
  {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
  {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
  {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}
};

// ==================== RETAINED STATE ====================
retained int DATA_INTERVAL_SECONDS = 3600;
retained bool initialBootComplete = false;
retained time_t lastConfigTime = 0;
retained time_t lastKnownTimestamp = 0;

// ==================== GLOBALS ====================
SdFat sd;
bool sdCardReady = false;

Adafruit_ADS1115 ads0(0x48);
Adafruit_ADS1115 ads1(0x49);
bool ads0Ready = false;
bool ads1Ready = false;

DS18 ds18(ONEWIRE_PIN);
bool cycleInProgress = false;

enum class TelemetryTransmitResult : uint8_t { Sent, Skipped, Failed };

// ==================== FUNCTION DECLARATIONS ====================
void initializePins();
void enableMicroSDPower();
void disableMicroSDPower();
void disableSPIPins();

void executeWakeCycle();
bool ensureParticleConnection(unsigned long timeoutMs);
bool waitForTimeSync(unsigned long timeoutMs);
bool requestThingsBoardConfig();
bool uploadQueuedTelemetry();
bool uploadSingleTelemetryPoint(const String &dataPoint);
bool buildTelemetryPayload(const String &dataPoint, char *payloadBuffer, size_t bufferSize, size_t &outLen);
bool publishTelemetryPayload(const char *payload);
TelemetryTransmitResult transmitTelemetryRecord(const String &record, bool allowSkipForQueue);
bool hasQueuedTelemetry();
void logDataPoint(String &dataPoint);
bool initSD();
void sleepUntilNextInterval();
void systemStatus(int status);
time_t resolveCurrentTimestamp();
bool isAddressConfigured(const uint8_t addr[8]);

bool httpPost(const char *host, int port, const char *path,
              const char *contentType, const char *body, size_t bodyLen,
              char *respBody, size_t respBodySize);

// ==================== SETUP ====================
void setup() {
  if (SERIAL_DEBUG) {
    Serial.begin(115200);
    waitFor(Serial.isConnected, 3000);
  }

  initializePins();

  Wire.begin();
  Wire.setClock(400000);

  pinMode(ONEWIRE_PIN, INPUT_PULLUP);
  ds18.setConversionTime(750);

  ads0.begin();
  ads0Ready = true;
  ads0.setGain(GAIN_ONE);

  ads1.begin();
  ads1Ready = true;
  ads1.setGain(GAIN_ONE);
  if (SERIAL_DEBUG) Serial.println("ADS1115 converters initialized");

  sdCardReady = initSD();

  if (SERIAL_DEBUG) Serial.println("STEMMNet Boron analog logger init complete");
}

// ==================== LOOP ====================
void loop() {
  if (!cycleInProgress) {
    cycleInProgress = true;
    executeWakeCycle();
    cycleInProgress = false;
  }
  Particle.process();
}

// ==================== CORE WAKE CYCLE ====================
void executeWakeCycle() {
  if (SERIAL_DEBUG) Serial.println("----- Wake cycle start -----");

  initializePins();

  time_t nowTs       = Time.now();
  bool timeIsValid   = Time.isValid();
  bool connectionOpened = ensureParticleConnection(
      initialBootComplete ? STANDARD_CONNECT_TIMEOUT_MS : FIRST_BOOT_CONNECT_TIMEOUT_MS);

  if (connectionOpened) {
    waitForTimeSync(60000);
    
    timeIsValid = Time.isValid();
    nowTs       = Time.now();

    bool shouldPullConfig = (!initialBootComplete) || 
                            (timeIsValid && Time.hour(nowTs) == 0 && Time.day(lastConfigTime) != Time.day(nowTs));

    if (shouldPullConfig) {
      if (requestThingsBoardConfig()) {
        initialBootComplete = true;
        if (timeIsValid) {
          lastConfigTime = nowTs;
        }
      } else if (SERIAL_DEBUG) {
        Serial.println("Config request failed (will retry next wake)");
      }
    }
  }

  String currentDataPoint;
  logDataPoint(currentDataPoint);

  bool uploadSuccess = false;

  if (sdCardReady) {
    if (!Particle.connected()) {
      if (ensureParticleConnection(STANDARD_CONNECT_TIMEOUT_MS)) {
        connectionOpened = true;
      }
    }
    if (Particle.connected()) {
      uploadSuccess = uploadQueuedTelemetry();
    }
  } else {
    if (currentDataPoint.length() > 0) {
      if (!Particle.connected()) {
        if (ensureParticleConnection(STANDARD_CONNECT_TIMEOUT_MS)) {
          connectionOpened = true;
        }
      }
      if (Particle.connected()) {
        uploadSuccess = uploadSingleTelemetryPoint(currentDataPoint);
      }
    }
  }

  if (Particle.connected() && connectionOpened) {
    Particle.disconnect();
  }

  Cellular.off();

  if (SERIAL_DEBUG) Serial.println("Preparing for sleep");

  sleepUntilNextInterval();

  if (SERIAL_DEBUG) Serial.println("----- Wake cycle end -----");
}

// ==================== CONNECTION HELPERS ====================
bool ensureParticleConnection(unsigned long timeoutMs) {
  if (Particle.connected()) return true;

  Particle.connect();
  unsigned long start = millis();
  while (!Particle.connected() && (millis() - start) < timeoutMs) {
    Particle.process();
    delay(100);
  }
  return Particle.connected();
}

bool timeIsValidFn() { return Time.isValid(); }

bool waitForTimeSync(unsigned long timeoutMs) {
  if (Time.isValid()) return true;
  return waitFor(timeIsValidFn, timeoutMs);
}

// ==================== CONFIG REQUEST ====================
bool requestThingsBoardConfig() {
  if (SERIAL_DEBUG) Serial.println("Requesting config via HTTP...");

  char respBody[16];
  bool ok = httpPost(INGEST_HOST, INGEST_PORT, CONFIG_PATH,
                     "text/plain", AUTH_TOKEN, strlen(AUTH_TOKEN),
                     respBody, sizeof(respBody));
  if (!ok) {
    if (SERIAL_DEBUG) Serial.println("Config HTTP request failed");
    return false;
  }

  String payload(respBody);
  payload.trim();
  if (payload.length() > 0) {
    int newInterval = payload.toInt();
    if (newInterval >= MIN_DATA_INTERVAL && newInterval <= MAX_DATA_INTERVAL) {
      DATA_INTERVAL_SECONDS = newInterval;
      if (SERIAL_DEBUG) Serial.printlnf("LOG_INTERVAL_SECONDS updated to: %d", DATA_INTERVAL_SECONDS);
      if (Time.isValid()) lastConfigTime = Time.now();
    } else if (SERIAL_DEBUG) {
      Serial.println("Config interval out of range; ignoring");
    }
  }

  if (SERIAL_DEBUG) Serial.println("Config received successfully");
  return true;
}

// ==================== TELEMETRY UPLOAD ====================
bool hasQueuedTelemetry() {
  if (!sdCardReady) return false;
  if (!sd.begin(MICROSD_CS)) {
    sdCardReady = false;
    return false;
  }
  return sd.exists("tmp_buffer.txt");
}

bool buildTelemetryPayload(const String &dataPoint, char *payloadBuffer, size_t bufferSize, size_t &outLen) {
  if (payloadBuffer == nullptr || bufferSize == 0) return false;

  ArduinoJson::DynamicJsonDocument readingDoc(2048);
  ArduinoJson::DeserializationError err = deserializeJson(readingDoc, dataPoint.c_str());
  if (err) {
    if (SERIAL_DEBUG) {
      Serial.print("Failed to parse telemetry JSON: ");
      Serial.println(err.c_str());
    }
    return false;
  }

  ArduinoJson::DynamicJsonDocument payloadDoc(3072);
  payloadDoc["token"] = AUTH_TOKEN;

  if (!readingDoc["ts"].isNull()) {
    payloadDoc["ts"] = readingDoc["ts"];
  }

  if (readingDoc["values"].is<ArduinoJson::JsonObject>()) {
    payloadDoc["values"] = readingDoc["values"];
  }

  outLen = serializeJson(payloadDoc, payloadBuffer, bufferSize);
  if (outLen == 0 || outLen >= bufferSize) {
    if (SERIAL_DEBUG) Serial.println("Telemetry payload serialization failed");
    return false;
  }

  payloadBuffer[outLen] = '\0';
  return true;
}

bool publishTelemetryPayload(const char *payload) {
  if (payload == nullptr || payload[0] == '\0') return false;

  if (SERIAL_DEBUG) Serial.println("Uploading telemetry via HTTP...");

  return httpPost(INGEST_HOST, INGEST_PORT, INGEST_PATH,
                  "application/json", payload, strlen(payload),
                  nullptr, 0);
}

TelemetryTransmitResult transmitTelemetryRecord(const String &record, bool allowSkipForQueue) {
  if (record.length() == 0) {
    return allowSkipForQueue ? TelemetryTransmitResult::Skipped : TelemetryTransmitResult::Failed;
  }

  if (record.indexOf("\"token\"") >= 0 && record.indexOf("\"values\"") >= 0) {
    if (publishTelemetryPayload(record.c_str())) {
      return TelemetryTransmitResult::Sent;
    }
    return TelemetryTransmitResult::Failed;
  }

  char payloadBuffer[POINT_LENGTH_BYTES];
  size_t payloadLen = 0;
  if (buildTelemetryPayload(record, payloadBuffer, sizeof(payloadBuffer), payloadLen)) {
    if (publishTelemetryPayload(payloadBuffer)) {
      return TelemetryTransmitResult::Sent;
    }
    return TelemetryTransmitResult::Failed;
  }

  if (allowSkipForQueue) {
    if (SERIAL_DEBUG) Serial.println("Skipping corrupt telemetry record in queue");
    return TelemetryTransmitResult::Skipped;
  }
  return TelemetryTransmitResult::Failed;
}

bool uploadQueuedTelemetry() {
  if (!sdCardReady) {
    if (SERIAL_DEBUG) Serial.println("SD not ready for queue upload");
    return false;
  }

  if (!sd.begin(MICROSD_CS)) {
    sdCardReady = false;
    if (SERIAL_DEBUG) Serial.println("SD begin failed during queue upload");
    return false;
  }

  if (!sd.exists("tmp_buffer.txt")) {
    if (SERIAL_DEBUG) Serial.println("No queued data to upload");
    return true;
  }

  SdFile queueFile;
  if (!queueFile.open("tmp_buffer.txt", FILE_READ)) {
    if (SERIAL_DEBUG) Serial.println("Failed to open tmp_buffer.txt");
    return false;
  }

  if (SERIAL_DEBUG) Serial.println("Uploading queued telemetry...");

  char buffer[POINT_LENGTH_BYTES];
  String lineAccumulator;
  lineAccumulator.reserve(MAX_QUEUE_LINE_BYTES);
  int uploadCount   = 0;
  int failCount     = 0;
  int skippedCount  = 0;
  bool encounteredError = false;

  while (!encounteredError) {
    int16_t readLen = queueFile.fgets(buffer, sizeof(buffer));
    if (readLen < 0) {
      failCount++;
      encounteredError = true;
      if (SERIAL_DEBUG) Serial.println("Error reading queued telemetry");
      break;
    }
    if (readLen == 0) break;

    bool hasNewline = buffer[readLen - 1] == '\n';
    if (hasNewline) buffer[readLen - 1] = '\0';

    lineAccumulator += buffer;

    if (lineAccumulator.length() > MAX_QUEUE_LINE_BYTES) {
      failCount++;
      encounteredError = true;
      if (SERIAL_DEBUG) Serial.println("Queued entry exceeded max length");
      break;
    }

    if (hasNewline) {
      String line = lineAccumulator;
      lineAccumulator = "";
      line.trim();

      TelemetryTransmitResult result = transmitTelemetryRecord(line, true);
      if (result == TelemetryTransmitResult::Sent) {
        uploadCount++;
        delay(1000); // Rate-limit uploads
      } else if (result == TelemetryTransmitResult::Skipped) {
        skippedCount++;
      } else {
        failCount++;
        encounteredError = true;
        if (SERIAL_DEBUG) Serial.println("Failed to upload queued point");
      }
    }
  }

  if (!encounteredError && lineAccumulator.length() > 0) {
    String line = lineAccumulator;
    line.trim();
    TelemetryTransmitResult result = transmitTelemetryRecord(line, true);
    if (result == TelemetryTransmitResult::Sent) {
      uploadCount++;
      delay(1000);
    } else if (result == TelemetryTransmitResult::Skipped) {
      skippedCount++;
    } else {
      failCount++;
      encounteredError = true;
      if (SERIAL_DEBUG) Serial.println("Failed to upload queued point");
    }
  }

  queueFile.close();

  if (failCount == 0) {
    if (!sd.remove("tmp_buffer.txt")) {
      SdFile truncFile;
      if (truncFile.open("tmp_buffer.txt", O_WRONLY | O_TRUNC)) {
        truncFile.close();
        if (SERIAL_DEBUG) Serial.println("tmp_buffer.txt truncated (remove failed)");
      } else if (SERIAL_DEBUG) {
        Serial.println("Warning: failed to remove or truncate tmp_buffer.txt");
      }
    }
    if (SERIAL_DEBUG) {
      Serial.printlnf("Successfully uploaded %d queued points%s",
                      uploadCount,
                      skippedCount > 0 ? " (with corrupt entries skipped)" : "");
    }
    return true;
  } else {
    if (SERIAL_DEBUG) Serial.printlnf("Partial upload: %d succeeded, %d failed", uploadCount, failCount);
    return false;
  }
}

bool uploadSingleTelemetryPoint(const String &dataPoint) {
  if (dataPoint.length() == 0) return false;
  if (SERIAL_DEBUG) Serial.println("Uploading single telemetry point...");
  TelemetryTransmitResult result = transmitTelemetryRecord(dataPoint, false);
  return result == TelemetryTransmitResult::Sent;
}

// ==================== DATA LOGGING ====================
bool isAddressConfigured(const uint8_t addr[8]) {
  for (size_t i = 0; i < 8; i++) {
    if (addr[i] != 0x00) return true;
  }
  return false;
}

void logDataPoint(String &dataPoint) {
  time_t nowTs = resolveCurrentTimestamp();

  if (SERIAL_DEBUG) Serial.println("Logging data point...");
  systemStatus(0);

  digitalWrite(SENSOR_POWER_PIN, HIGH);
  delay(1000);

  ArduinoJson::DynamicJsonDocument doc(2048);
  doc["ts"] = static_cast<uint64_t>(nowTs) * 1000ULL;
  ArduinoJson::JsonObject values = doc["values"].to<ArduinoJson::JsonObject>();

  for (int i = 0; i < MAX_DS18_SENSORS; i++) {
    char key[10];
    snprintf(key, sizeof(key), "temp%d", i);
    if (!isAddressConfigured(dsAddresses[i])) {
      values[key] = -99.99;
      continue;
    }

    bool readOk = ds18.read(dsAddresses[i]);
    if (!readOk) {
      if (SERIAL_DEBUG) {
        if (ds18.crcError()) {
          Serial.print("CRC error reading sensor ");
          Serial.println(i);
        } else {
          Serial.print("Failed to read DS18 sensor ");
          Serial.println(i);
        }
      }
      values[key] = -99.99;
      continue;
    }

    values[key] = ds18.celsius();
  }

  for (int i = 0; i < 4; i++) {
    char key[16];
    snprintf(key, sizeof(key), "adc0_c%d", i);
    if (ads0Ready) {
      int16_t raw = ads0.readADC_SingleEnded(i);
      float voltage = raw * ADS1115_MV_PER_BIT_GAIN_ONE;
      values[key] = voltage;
    } else {
      values[key] = -998.0;
    }
  }

  for (int i = 0; i < 4; i++) {
    char key[16];
    snprintf(key, sizeof(key), "adc1_c%d", i);
    if (ads1Ready) {
      int16_t raw = ads1.readADC_SingleEnded(i);
      float voltage = raw * ADS1115_MV_PER_BIT_GAIN_ONE;
      values[key] = voltage;
    } else {
      values[key] = -998.0;
    }
  }

  char buffer[POINT_LENGTH_BYTES];
  size_t len = serializeJson(doc, buffer, sizeof(buffer));
  if (len >= sizeof(buffer)) {
    buffer[sizeof(buffer) - 1] = '\0';
  } else {
    buffer[len] = '\0';
  }
  dataPoint = String(buffer);

  digitalWrite(SENSOR_POWER_PIN, LOW);

  if (SERIAL_DEBUG) {
    Serial.print("Data: ");
    Serial.println(dataPoint);
  }

  if (sdCardReady) {
    if (!sd.begin(MICROSD_CS)) {
      sdCardReady = false;
      if (SERIAL_DEBUG) Serial.println("SD begin failed during log");
    } else {
      char dailyPath[20];
      if (Time.isValid()) {
        sprintf(dailyPath, "%04d_%02d_%02d.txt", Time.year(nowTs), Time.month(nowTs), Time.day(nowTs));
      } else {
        sprintf(dailyPath, "offline.txt");
      }

      SdFile dailyFile;
      if (dailyFile.open(dailyPath, FILE_WRITE)) {
        dailyFile.println(dataPoint);
        dailyFile.close();
        if (SERIAL_DEBUG) {
          Serial.print("Saved to ");
          Serial.println(dailyPath);
        }
      } else {
        if (SERIAL_DEBUG) Serial.println("Failed to open daily file");
      }

      SdFile bufferFile;
      if (bufferFile.open("tmp_buffer.txt", FILE_WRITE)) {
        char queuePayload[POINT_LENGTH_BYTES];
        size_t queuePayloadLen = 0;
        bool payloadPrepared = buildTelemetryPayload(dataPoint, queuePayload, sizeof(queuePayload), queuePayloadLen);

        if (payloadPrepared) {
          bufferFile.write(queuePayload, queuePayloadLen);
          bufferFile.write('\n');
        } else {
          bufferFile.println(dataPoint);
        }

        bufferFile.close();

        if (SERIAL_DEBUG) {
          if (payloadPrepared) Serial.println("Added telemetry payload to upload queue");
          else Serial.println("Added raw reading to upload queue");
        }
      } else {
        sdCardReady = false;
        if (SERIAL_DEBUG) Serial.println("Failed to open tmp_buffer.txt");
      }
    }
  }

  systemStatus(1);
}

time_t resolveCurrentTimestamp() {
  if (Time.isValid()) {
    lastKnownTimestamp = Time.now();
    return lastKnownTimestamp;
  }
  if (lastKnownTimestamp > 0) {
    lastKnownTimestamp += DATA_INTERVAL_SECONDS;
    return lastKnownTimestamp;
  }
  lastKnownTimestamp = Time.now();
  return lastKnownTimestamp;
}

bool initSD() {
  enableMicroSDPower();
  delay(50);
  if (sd.begin(MICROSD_CS)) {
    if (SERIAL_DEBUG) Serial.println("SD card initialized");
    return true;
  } else {
    if (SERIAL_DEBUG) Serial.println("SD card init failed");
    return false;
  }
}

void sleepUntilNextInterval() {
  disableMicroSDPower();
  disableSPIPins();

  time_t reference = Time.isValid() ? Time.now() : lastKnownTimestamp;
  if (reference <= 0) reference = lastKnownTimestamp;

  time_t nextLog = ((reference / DATA_INTERVAL_SECONDS) + 1) * DATA_INTERVAL_SECONDS;
  time_t sleepSeconds = nextLog - reference - 10;
  if (sleepSeconds < 10) sleepSeconds = DATA_INTERVAL_SECONDS;

  if (SERIAL_DEBUG) Serial.printlnf("Sleeping for %ld seconds", (long)sleepSeconds);

  SystemSleepConfiguration sleepConfig;
  sleepConfig.mode(SystemSleepMode::ULTRA_LOW_POWER);
  sleepConfig.duration(std::chrono::seconds(sleepSeconds));
  System.sleep(sleepConfig);
}

void systemStatus(int status) {
  switch (status) {
    case 0: // Blink
      digitalWrite(STATUS_LED_PIN, HIGH);
      delay(50);
      digitalWrite(STATUS_LED_PIN, LOW);
      break;
    case 1: // Success
      for (int i = 0; i < 3; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(50);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(100);
      }
      break;
    case 2: // Error
      digitalWrite(STATUS_LED_PIN, HIGH);
      delay(500);
      digitalWrite(STATUS_LED_PIN, LOW);
      delay(500);
      break;
  }
}

void initializePins() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, LOW);

  enableMicroSDPower();
}

void enableMicroSDPower() {
  if (SD_CARD_POWER_PIN != PIN_INVALID) {
    pinMode(SD_CARD_POWER_PIN, OUTPUT);
    digitalWrite(SD_CARD_POWER_PIN, LOW);
  }
  pinMode(MICROSD_CS, OUTPUT);
  digitalWrite(MICROSD_CS, HIGH);
}

void disableMicroSDPower() {
  if (SD_CARD_POWER_PIN != PIN_INVALID) {
    digitalWrite(SD_CARD_POWER_PIN, HIGH);
  }
}

void disableSPIPins() {
  pinMode(MICROSD_CS, OUTPUT);
  digitalWrite(MICROSD_CS, HIGH);

  pinMode(MISO, INPUT_PULLDOWN);
  pinMode(MOSI, INPUT_PULLDOWN);
  pinMode(SCK, INPUT_PULLDOWN);
}

// ==================== HTTP CLIENT ====================
bool httpPost(const char *host, int port, const char *path,
              const char *contentType, const char *body, size_t bodyLen,
              char *respBody, size_t respBodySize) {
  TCPClient client;
  if (!client.connect(host, port)) {
    if (SERIAL_DEBUG) Serial.printlnf("HTTP: connect failed (%s:%d)", host, port);
    return false;
  }

  client.print("POST "); client.print(path); client.println(" HTTP/1.0");
  client.print("Host: "); client.println(host);
  client.print("Content-Type: "); client.println(contentType);
  client.print("Content-Length: "); client.println((int)bodyLen);
  client.println("Connection: close");
  client.println(); 
  client.write((const uint8_t*)body, bodyLen);

  unsigned long start = millis();
  while (!client.available() && (millis() - start) < HTTP_RESPONSE_TIMEOUT_MS) {
    delay(10);
  }
  if (!client.available()) {
    if (SERIAL_DEBUG) Serial.println("HTTP: response timeout");
    client.stop();
    return false;
  }

  char statusLine[64];
  int si = 0;
  while (client.available() && si < (int)sizeof(statusLine) - 1) {
    char c = client.read();
    if (c == '\n') break;
    if (c != '\r') statusLine[si++] = c;
  }
  statusLine[si] = '\0';

  int statusCode = 0;
  const char *sp = strchr(statusLine, ' ');
  if (sp) statusCode = atoi(sp + 1);
  bool success = statusCode >= 200 && statusCode < 300;

  if (SERIAL_DEBUG) Serial.printlnf("HTTP: POST %s -> %d", path, statusCode);

  int newlines = 0;
  while (client.available()) {
    char c = client.read();
    if (c == '\r') continue;
    if (c == '\n') { if (++newlines == 2) break; }
    else            { newlines = 0; }
  }

  if (respBody && respBodySize > 0) {
    size_t ri = 0;
    while (client.available() && ri < respBodySize - 1) {
      respBody[ri++] = client.read();
    }
    respBody[ri] = '\0';
  }

  client.stop();
  return success;
}
