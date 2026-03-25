/*
 * SN2-Particle-SDI12
 * STEMMNet Station v2 — Particle Boron — SDI-12 Sensor Array
 *
 * Features:
 * - Particle Boron with SEMI_AUTOMATIC / SYSTEM_THREAD connection mode
 * - ThingsBoard telemetry via direct HTTP POST (plain HTTP, port 80)
 * - ThingsBoard config download via direct HTTP POST (/ingest/config.php)
 *   - Configurable LOG_INTERVAL_SECONDS (persisted across sleep via retained)
 * - SDI-12 sensor reading on D4 (addresses 0-9, up to 10 sensors)
 *   - Sensor discovery persisted across sleep cycles via retained variables
 *   - Per-sensor raw data string stored as "sdi12_a0" ... "sdi12_a9"
 * - MicroSD card queueing (tmp_buffer.txt) with daily log files
 * - Upload triggered every wake cycle (same interval as logging)
 * - Ultra-low-power sleep between intervals
 * - SENSOR_POWER_PIN (D2) powers the SDI-12 bus and sensors
 *
 * Written by: Nick Perlaky
 */

#include "Particle.h"

#define ARDUINOJSON_ENABLE_PROGMEM 0
#include <ArduinoJson.h>
#include <SdFat.h>
#include <SDI12.h>
#include <Adafruit_ADS1X15.h>

SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);
#include <chrono>

// ==================== PIN CONFIG ====================
constexpr pin_t MICROSD_CS        = SS;          // D14 / A5
constexpr pin_t SENSOR_POWER_PIN  = D2;
constexpr pin_t STATUS_LED_PIN    = D7;
constexpr pin_t SDI12_DATA_PIN    = D4;          // Same pin as OneWire in STEMMNet original
constexpr pin_t SD_CARD_POWER_PIN = PIN_INVALID; // Update if carrier exposes dedicated power enable

// ==================== CONFIGURATION ====================
constexpr bool SERIAL_DEBUG = true;

// Station identification — update token for each deployed station
const char THINGSBOARD_TOKEN[]  = "a53aavt8hb8dkodwa2i2";
const char CONFIG_SHARED_KEYS[] = "LOG_INTERVAL_SECONDS";

constexpr size_t POINT_LENGTH_BYTES        = 1024;  // Increased vs original to fit SDI-12 data
constexpr size_t MAX_QUEUE_LINE_BYTES      = 8192;
constexpr unsigned long FIRST_BOOT_CONNECT_TIMEOUT_MS  = 5UL * 60UL * 1000UL;
constexpr unsigned long STANDARD_CONNECT_TIMEOUT_MS    = 60UL * 1000UL;
constexpr unsigned long HTTP_RESPONSE_TIMEOUT_MS       = 15000UL;

// Direct HTTP upload — plain HTTP (port 80) to avoid TLS overhead on cellular
const char INGEST_HOST[] = "api.alclimate.com";
constexpr int  INGEST_PORT  = 80;
const char INGEST_PATH[] = "/ingest/auburnh2onet.php";
const char CONFIG_PATH[]  = "/ingest/config.php";
constexpr int MIN_DATA_INTERVAL  = 60;      // 1 minute
constexpr int MAX_DATA_INTERVAL  = 86400;   // 24 hours

// SDI-12 configuration
constexpr int MAX_SDI12_SENSORS  = 10;  // Addresses '0' through '9'
constexpr int SDI12_RESPONSE_TIMEOUT_MS = 500;
constexpr int SDI12_MEAS_EXTRA_WAIT_MS  = 1000;  // Extra buffer after nominal wait

// ==================== RETAINED STATE ====================
// Persists across ultra-low-power sleep cycles
retained int  DATA_INTERVAL_SECONDS    = 3600; // Default 1 hour; overridden by ThingsBoard config
retained bool initialBootComplete      = false;
retained time_t lastConfigTime         = 0;
retained time_t lastKnownTimestamp     = 0;

// SDI-12 sensor discovery — persisted so we don't re-scan every wake cycle
retained bool sensorsDiscovered        = false;
retained int  numActiveSensors         = 0;
retained char activeSensorAddresses[MAX_SDI12_SENSORS];

// ==================== GLOBALS ====================
SdFat sd;
bool sdCardReady = false;

SDI12 mySDI12(SDI12_DATA_PIN);
Adafruit_ADS1115 ads0;

bool cycleInProgress = false;


enum class TelemetryTransmitResult : uint8_t {
  Sent,
  Skipped,
  Failed
};

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

bool httpPost(const char *host, int port, const char *path,
              const char *contentType, const char *body, size_t bodyLen,
              char *respBody, size_t respBodySize);

// SDI-12
void discoverSDI12Sensors();
bool checkSDI12Active(char address);
bool readSDI12Sensor(char address, String &data);
String readSDI12Response(uint32_t timeoutMs = SDI12_RESPONSE_TIMEOUT_MS);
char decToChar(byte i);
byte charToDec(char i);

// ==================== SETUP ====================
void setup() {
  if (SERIAL_DEBUG) {
    Serial.begin(115200);
    waitFor(Serial.isConnected, 3000);
  }

  initializePins();

  // Initialize SDI-12 bus
  mySDI12.begin();

  // Initialize ADS1115
  ads0.begin();
  ads0.setGain(GAIN_ONE);

  sdCardReady = initSD();

  if (SERIAL_DEBUG) {
    Serial.println("SN2-Particle-SDI12 init complete");
    Serial.printlnf("Sensors previously discovered: %s (%d sensors)",
                    sensorsDiscovered ? "yes" : "no", numActiveSensors);
  }
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
  if (SERIAL_DEBUG) {
    Serial.println("----- Wake cycle start -----");
  }

  initializePins(); // Ensure pins are in known states each wake

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

  // Discover SDI-12 sensors on first boot (after time sync so power is stable)
  if (!sensorsDiscovered) {
    discoverSDI12Sensors();
    sensorsDiscovered = true;
  }

  // Log data point
  String currentDataPoint;
  logDataPoint(currentDataPoint);

  // Upload every time we log
  bool uploadSuccess = false;

  if (sdCardReady) {
    // If SD card is working, upload queued data (includes current point)
    if (!Particle.connected()) {
      if (ensureParticleConnection(STANDARD_CONNECT_TIMEOUT_MS)) {
        connectionOpened = true;
      }
    }
    if (Particle.connected()) {
      uploadSuccess = uploadQueuedTelemetry();
    }
  } else {
    // If SD failed, upload single current point immediately
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

  if (SERIAL_DEBUG) {
    Serial.println("Preparing for sleep");
  }

  sleepUntilNextInterval();

  if (SERIAL_DEBUG) {
    Serial.println("----- Wake cycle end -----");
  }
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

bool timeIsValidFn() {
  return Time.isValid();
}

bool waitForTimeSync(unsigned long timeoutMs) {
  if (Time.isValid()) return true;
  return waitFor(timeIsValidFn, timeoutMs);
}

// ==================== CONFIG REQUEST ====================
bool requestThingsBoardConfig() {
  if (SERIAL_DEBUG) Serial.println("Requesting config via HTTP...");

  char respBody[16];
  bool ok = httpPost(INGEST_HOST, INGEST_PORT, CONFIG_PATH,
                     "text/plain", THINGSBOARD_TOKEN, strlen(THINGSBOARD_TOKEN),
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

  ArduinoJson::DynamicJsonDocument readingDoc(3072);
  ArduinoJson::DeserializationError err = deserializeJson(readingDoc, dataPoint.c_str());
  if (err) {
    if (SERIAL_DEBUG) {
      Serial.print("Failed to parse telemetry JSON: ");
      Serial.println(err.c_str());
    }
    return false;
  }

  ArduinoJson::DynamicJsonDocument payloadDoc(4096);
  payloadDoc["token"] = THINGSBOARD_TOKEN;

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

  // If already a fully-formed payload (contains token + values), publish directly
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
    if (readLen == 0) break; // EOF

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

  // Flush any remaining partial line
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
    }
  }

  queueFile.close();

  if (failCount == 0) {
    if (!sd.remove("tmp_buffer.txt")) {
      // Removal failed — truncate so uploaded records aren't re-sent next cycle.
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
    if (SERIAL_DEBUG) {
      Serial.printlnf("Partial upload: %d succeeded, %d failed", uploadCount, failCount);
    }
    return false;
  }
}

bool uploadSingleTelemetryPoint(const String &dataPoint) {
  if (dataPoint.length() == 0) return false;
  if (SERIAL_DEBUG) Serial.println("Uploading single telemetry point...");
  TelemetryTransmitResult result = transmitTelemetryRecord(dataPoint, false);
  return result == TelemetryTransmitResult::Sent;
}

// ==================== SDI-12 DISCOVERY ====================
/**
 * Discover SDI-12 sensors at addresses '0' through '9'.
 * Results are stored in the retained arrays activeSensorAddresses / numActiveSensors
 * and therefore only need to run once per power cycle.
 */
void discoverSDI12Sensors() {
  if (SERIAL_DEBUG) Serial.println("Discovering SDI-12 sensors (addresses 0-9)...");

  // Power on sensors
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  delay(1000); // Allow sensors to boot (some DDI sensors need up to 1 s)

  numActiveSensors = 0;

  for (byte i = 0; i < 10; i++) {
    char addr = decToChar(i);

    if (SERIAL_DEBUG) {
      Serial.printlnf("  Polling address '%c'...", addr);
    }

    if (checkSDI12Active(addr)) {
      activeSensorAddresses[numActiveSensors] = addr;
      numActiveSensors++;
      if (SERIAL_DEBUG) {
        Serial.printlnf("  Found sensor at address '%c'", addr);
      }
    } else {
      if (SERIAL_DEBUG) Serial.println("  No response");
    }

    delay(100); // Brief gap between polls
  }

  if (SERIAL_DEBUG) {
    Serial.printlnf("SDI-12 discovery complete: %d sensor(s) found", numActiveSensors);
  }

  digitalWrite(SENSOR_POWER_PIN, LOW);
}

/**
 * Check whether a sensor is present at the given SDI-12 address.
 * Uses the acknowledge command (a!) followed by the identify command (aI!)
 * to confirm a valid sensor (not just line noise).
 * Retries up to 3 times before giving up.
 */
bool checkSDI12Active(char address) {
  String ackCommand  = String(address) + "!";
  String infoCommand = String(address) + "I!";

  for (int attempt = 0; attempt < 3; attempt++) {
    // Send acknowledge command
    mySDI12.clearBuffer();
    mySDI12.sendCommand(ackCommand.c_str());

    // Wait for response using helper
    String probeResp = readSDI12Response();

    if (probeResp.length() > 0) {
      // Got an acknowledge — now send identify command to confirm it's a real sensor
      mySDI12.clearBuffer();
      mySDI12.sendCommand(infoCommand.c_str());

      String infoResp = readSDI12Response();

      // A real SDI-12 sensor's I! response is at least 20 characters
      if (infoResp.length() >= 20) {
        mySDI12.clearBuffer();
        return true;
      }
    }
  }

  mySDI12.clearBuffer();
  return false;
}

// ==================== SDI-12 SENSOR READING ====================
/**
 * Read a single SDI-12 sensor using the standard M! / D0! measurement sequence.
 *
 * @param address  The sensor address character ('0'-'9')
 * @param data     Output string containing the raw sensor values (e.g. "+22.5+18.3+0.542")
 * @return true on success, false on timeout or empty response
 *
 * The raw values string starts from after the address character in the D0! response.
 * Multiple values from the same sensor are concatenated in the single string exactly as
 * returned by the sensor, so they can be stored compactly in the JSON payload.
 */
bool readSDI12Sensor(char address, String &data) {
  data = "-999"; // Sentinel failure value

  // --- Step 1: Send M! (start measurement) ---
  String mCommand = String(address) + "M!";
  mySDI12.clearBuffer();
  mySDI12.sendCommand(mCommand.c_str());

  String mResponse = readSDI12Response();

  // M! response format: atttn  (a=address, ttt=wait seconds, n=num values)
  if (mResponse.length() < 4) {
    if (SERIAL_DEBUG) {
      Serial.printlnf("Invalid M! response for sensor '%c': '%s'", address, mResponse.c_str());
    }
    return false;
  }

  int waitSeconds = mResponse.substring(1, 4).toInt();
  int numValues   = mResponse.substring(4).toInt();

  if (numValues == 0) {
    if (SERIAL_DEBUG) {
      Serial.printlnf("Sensor '%c' reports 0 values", address);
    }
    return false;
  }

  if (SERIAL_DEBUG) {
    Serial.printlnf("Sensor '%c' M! resp: '%s' (wait %ds, %d value(s))",
                    address, mResponse.c_str(), waitSeconds, numValues);
  }

  // --- Step 2: Wait for measurement to complete ---
  unsigned long waitMs = (unsigned long)(waitSeconds * 1000) + SDI12_MEAS_EXTRA_WAIT_MS;
  unsigned long waitStart = millis();
  while ((millis() - waitStart) < waitMs) {
    if (mySDI12.available()) {
      mySDI12.clearBuffer(); // Discard any intermediate traffic (e.g. service request)
    }
    delay(10);
  }

  // --- Step 3: Send D0! (retrieve data) ---
  String dCommand = String(address) + "D0!";
  mySDI12.clearBuffer();
  mySDI12.sendCommand(dCommand.c_str());

  String dResponse = readSDI12Response();

  if (dResponse.length() > 1) {
    // First character is the echoed address — strip it
    data = dResponse.substring(1);
    if (SERIAL_DEBUG) {
      Serial.printlnf("Sensor '%c' data: '%s'", address, data.c_str());
    }
    return true;
  }

  if (SERIAL_DEBUG) {
    Serial.printlnf("No D0! data for sensor '%c': '%s'", address, dResponse.c_str());
  }
  return false;
}

/**
 * Read all available SDI-12 response characters into a string, with 30ms inter-character timeout.
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

// ==================== DATA LOGGING ====================
/**
 * Read all discovered SDI-12 sensors and build a JSON telemetry data point.
 *
 * JSON structure (stored in dataPoint and queued to SD):
 * {
 *   "ts": <unix_ms>,
 *   "values": {
 *     "sdi12_a0": "+22.5+18.3+0.542",   // raw value string from sensor 0
 *     "sdi12_a1": "-999",                // failed read
 *     ...
 *   }
 * }
 *
 * The raw value strings preserve the full SDI-12 response for flexibility — the
 * ThingsBoard data converter or post-processing pipeline can split them into individual
 * channels if needed.
 */
void logDataPoint(String &dataPoint) {
  time_t nowTs = resolveCurrentTimestamp();

  if (SERIAL_DEBUG) Serial.println("Logging data point...");
  systemStatus(0);

  // Power on sensor bus
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  delay(1000); // Allow sensors to stabilize

  ArduinoJson::DynamicJsonDocument doc(3072);
  doc["ts"] = static_cast<uint64_t>(nowTs) * 1000ULL;
  ArduinoJson::JsonObject values = doc["values"].to<ArduinoJson::JsonObject>();

  // Read ADS1115 channels (0-3)
  for (int i = 0; i < 4; i++) {
    // GAIN_ONE is +/- 4.096V max, meaning 1 bit = 0.125mV
    float voltage = ads0.readADC_SingleEnded(i) * 0.125F;
    char key[12];
    snprintf(key, sizeof(key), "adc0_c%d", i);
    values[key] = voltage;
  }

  for (int i = 0; i < numActiveSensors; i++) {
    char addr = activeSensorAddresses[i];
    char key[12];
    snprintf(key, sizeof(key), "sdi12_a%c", addr);

    String sensorData;
    if (readSDI12Sensor(addr, sensorData)) {
      values[key] = sensorData;
    } else {
      values[key] = "-999";
    }
  }

  // Power off sensor bus
  digitalWrite(SENSOR_POWER_PIN, LOW);

  char buffer[POINT_LENGTH_BYTES];
  size_t len = serializeJson(doc, buffer, sizeof(buffer));
  if (len >= sizeof(buffer)) {
    buffer[sizeof(buffer) - 1] = '\0';
    if (SERIAL_DEBUG) Serial.println("Warning: data point truncated (buffer too small)");
  } else {
    buffer[len] = '\0';
  }
  dataPoint = String(buffer);

  if (SERIAL_DEBUG) {
    Serial.print("Data: ");
    Serial.println(dataPoint);
  }

  // Save to SD card
  if (sdCardReady) {
    if (!sd.begin(MICROSD_CS)) {
      sdCardReady = false;
      if (SERIAL_DEBUG) Serial.println("SD begin failed during log");
    } else {
      // Save to daily file (YYYY_MM_DD.txt)
      char dailyPath[20];
      if (Time.isValid()) {
        sprintf(dailyPath, "%04d_%02d_%02d.txt",
                Time.year(nowTs), Time.month(nowTs), Time.day(nowTs));
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
        if (SERIAL_DEBUG) Serial.println("Failed to open daily log file");
      }

      // Also save a fully-formed payload to the upload buffer
      SdFile bufferFile;
      if (bufferFile.open("tmp_buffer.txt", FILE_WRITE)) {
        char queuePayload[POINT_LENGTH_BYTES];
        size_t queuePayloadLen = 0;
        bool payloadPrepared = buildTelemetryPayload(dataPoint, queuePayload,
                                                     sizeof(queuePayload), queuePayloadLen);
        if (payloadPrepared) {
          bufferFile.write(queuePayload, queuePayloadLen);
          bufferFile.write('\n');
        } else {
          // Fallback: write raw data point; transmitTelemetryRecord will wrap it on upload
          bufferFile.println(dataPoint);
        }
        bufferFile.close();

        if (SERIAL_DEBUG) {
          Serial.println(payloadPrepared ? "Added payload to upload queue"
                                        : "Added raw reading to upload queue");
        }
      } else {
        sdCardReady = false;
        if (SERIAL_DEBUG) Serial.println("Failed to open tmp_buffer.txt");
      }
    }
  }

  systemStatus(1);
}

// ==================== TIMESTAMP RESOLUTION ====================
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

// ==================== SD CARD ====================
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

// ==================== SLEEP ====================
void sleepUntilNextInterval() {
  disableMicroSDPower();
  disableSPIPins();

  time_t reference = Time.isValid() ? Time.now() : lastKnownTimestamp;
  if (reference <= 0) reference = lastKnownTimestamp;

  time_t nextLog    = ((reference / DATA_INTERVAL_SECONDS) + 1) * DATA_INTERVAL_SECONDS;
  time_t sleepSeconds = nextLog - reference - 10;
  if (sleepSeconds < 10) sleepSeconds = DATA_INTERVAL_SECONDS;

  if (SERIAL_DEBUG) {
    Serial.printlnf("Sleeping for %ld seconds", (long)sleepSeconds);
  }

  SystemSleepConfiguration sleepConfig;
  sleepConfig.mode(SystemSleepMode::ULTRA_LOW_POWER);
  sleepConfig.duration(std::chrono::seconds(sleepSeconds));
  System.sleep(sleepConfig);
}

// ==================== STATUS LED ====================
void systemStatus(int status) {
  switch (status) {
    case 0: // Single blink — measurement starting
      digitalWrite(STATUS_LED_PIN, HIGH);
      delay(50);
      digitalWrite(STATUS_LED_PIN, LOW);
      break;
    case 1: // Triple blink — success
      for (int i = 0; i < 3; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(50);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(100);
      }
      break;
    case 2: // Long blink — error
      digitalWrite(STATUS_LED_PIN, HIGH);
      delay(500);
      digitalWrite(STATUS_LED_PIN, LOW);
      delay(500);
      break;
  }
}

// ==================== PIN MANAGEMENT ====================
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
    digitalWrite(SD_CARD_POWER_PIN, LOW); // Assumed active-low
  }
  pinMode(MICROSD_CS, OUTPUT);
  digitalWrite(MICROSD_CS, HIGH);
}

void disableMicroSDPower() {
  if (SD_CARD_POWER_PIN != PIN_INVALID) {
    digitalWrite(SD_CARD_POWER_PIN, HIGH); // Assumed active-low
  }
}

void disableSPIPins() {
  pinMode(MICROSD_CS, OUTPUT);
  digitalWrite(MICROSD_CS, HIGH);
  pinMode(MISO, INPUT_PULLDOWN);
  pinMode(MOSI, INPUT_PULLDOWN);
  pinMode(SCK,  INPUT_PULLDOWN);
}

// ==================== HTTP CLIENT ====================
/**
 * Perform a synchronous HTTP POST. Uses TCPClient which works over the
 * cellular IP stack independently of the Particle cloud connection.
 *
 * @param host          Server hostname or IP
 * @param port          TCP port (80 for plain HTTP)
 * @param path          Request path (e.g. "/ingest/stemmnet.php")
 * @param contentType   Content-Type header value
 * @param body          Request body bytes
 * @param bodyLen       Length of body
 * @param respBody      Buffer to receive response body (may be nullptr)
 * @param respBodySize  Size of respBody buffer
 * @return true if server responded with HTTP 2xx
 */
bool httpPost(const char *host, int port, const char *path,
              const char *contentType, const char *body, size_t bodyLen,
              char *respBody, size_t respBodySize) {
  TCPClient client;
  if (!client.connect(host, port)) {
    if (SERIAL_DEBUG) Serial.printlnf("HTTP: connect failed (%s:%d)", host, port);
    return false;
  }

  // -- Send request --
  client.print("POST "); client.print(path); client.println(" HTTP/1.0");
  client.print("Host: "); client.println(host);
  client.print("Content-Type: "); client.println(contentType);
  client.print("Content-Length: "); client.println((int)bodyLen);
  client.println("Connection: close");
  client.println(); // blank line ends headers
  client.write((const uint8_t*)body, bodyLen);

  // -- Wait for response --
  unsigned long start = millis();
  while (!client.available() && (millis() - start) < HTTP_RESPONSE_TIMEOUT_MS) {
    delay(10);
  }
  if (!client.available()) {
    if (SERIAL_DEBUG) Serial.println("HTTP: response timeout");
    client.stop();
    return false;
  }

  // -- Parse status line (e.g. "HTTP/1.0 200 OK") --
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

  // -- Skip remaining headers (read until blank line) --
  int newlines = 0;
  while (client.available()) {
    char c = client.read();
    if (c == '\r') continue;
    if (c == '\n') { if (++newlines == 2) break; }
    else            { newlines = 0; }
  }

  // -- Capture response body if caller wants it --
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

// ==================== UTILITY ====================
/**
 * Convert a decimal integer (0-9) to its SDI-12 address character.
 * Returns '?' for values outside 0-9.
 */
char decToChar(byte i) {
  if (i < 10) return i + '0';
  return '?';
}

/**
 * Convert an SDI-12 address character ('0'-'9') to its decimal value.
 * Returns 255 for invalid characters.
 */
byte charToDec(char i) {
  if (i >= '0' && i <= '9') return i - '0';
  return 255;
}
