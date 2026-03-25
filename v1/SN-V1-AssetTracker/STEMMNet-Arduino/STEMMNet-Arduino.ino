/*
 * 
 *     UAH STEMMNet — Arduino
 *     
 *   Written by Nick Perlaky 2024
 *   
 *   Revision 02/19/2024
 * 
 */

#include <SparkFun_u-blox_SARA-R5_Arduino_Library.h>
#include "AssetTrackerPins.h"
#include <RTClib.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_ADS1X15.h>
#include <IPAddress.h>
#include <SPI.h>
#include <SdFat.h>

#define SERIAL_DEBUG 1

/****************************/

#define SN_ID "SN######"
#define API_KEY ""
RTC_DATA_ATTR int LOG_INTERVAL_SECONDS = 300;
RTC_DATA_ATTR int UPLOAD_INTERVAL_SECONDS = 10800;
#define CONFIG_INTERVAL_SECONDS 86400

// Temperature sensor addresses - in order of 0->4
DeviceAddress dsAddresses[5] = {
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}
};

/****************************/

// Hardware constants
#define LED_PIN LED_BUILTIN//PWM0
#define POWER_MOSFET_PIN PWM0
#define CONFIG_PIN D1
#define DS_PIN D0

// Data logging constants
#define SLEEP_INTERVAL_SECONDS 5
#define LOG_BUFFER_SECONDS 15
#define CELL_CONNECT_TIMEOUT_MS 120000
#define POINT_LENGTH_BYTES 111 // 110 bytes + ; + \0

// Network constants
String STATION_ID = SN_ID;
String SERVER_API_KEY = API_KEY;
String SERVER_URL = "data.alclimate.com";
#define SERVER_PORT 1000

// Status constants
#define STATUS_BLINK 0
#define STATUS_SUCCESS 1
#define STATUS_ERROR 2

// Clock
RTC_DS3231 rtc;
DateTime currentTime;
uint32_t tempTime;

// SARA R5
#define saraSerial Serial1
SARA_R5 assetTracker(SARA_PWR);
PositionData gps;
SpeedData spd;
ClockData clk;
boolean valid;
boolean radioWasOn = false;
RTC_DATA_ATTR boolean radioWasInitiallyTurnedOff = false;
boolean uploadComplete = false;


// OneWire
OneWire oneWire(DS_PIN);
DallasTemperature dsSensors(&oneWire);


// ADS
Adafruit_ADS1115 ads0;
Adafruit_ADS1115 ads1;


// SD
SdFat sd;
SdFile logFile;
boolean sdCardFunctioning = false;


/* Main setup */
void setup() {

  
  /* Start serial */
  if (SERIAL_DEBUG) {Serial.begin(115200);delay(100);Serial.println("Booted...");}


  /* Hardware setup */
  initializeAssetTrackerPins();
  assetTracker.invertPowerPin(true);
  pinMode(LED_PIN, OUTPUT);
  pinMode(POWER_MOSFET_PIN, OUTPUT);
  pinMode(DS_PIN, INPUT);


  /* RTC Initialization */
  while (!rtc.begin()) {
    if (SERIAL_DEBUG) Serial.println(F("RTC Error..."));
    systemStatus(STATUS_ERROR);
  }

  // Check if time is below 1 December 2023 (arbitrarily selected) ... if so need to run config
  currentTime = rtc.now();
  if (SERIAL_DEBUG) {Serial.print("Current time: ");Serial.println(currentTime.unixtime());}
  if (!radioWasInitiallyTurnedOff) {
    if (SERIAL_DEBUG) Serial.println(F("Pulling config data..."));
    cellRadioSetup();
    timeConfig();
    systemSleep(SLEEP_INTERVAL_SECONDS);
  }

  /* Log time check */
  if (SERIAL_DEBUG) {Serial.print("Current time: ");Serial.println(currentTime.unixtime());}
  if ( (currentTime.unixtime() % LOG_INTERVAL_SECONDS) - LOG_INTERVAL_SECONDS < -LOG_BUFFER_SECONDS ) {
    systemStatus(STATUS_BLINK);
    systemSleep(SLEEP_INTERVAL_SECONDS);
  } // Exits to sleep if not on an interval


  /*******************************************/
  /* BELOW runs if interval has been reached */
  /*******************************************/


  /* SD Card Check */
  enableMicroSDPower();
  SD_and_IMU_SPI.begin();
  if (sd.begin(MICROSD_CS, SD_SCK_MHZ(24)) == false) {
    sdCardFunctioning = false;
    if (SERIAL_DEBUG) Serial.println(F("SD Error..."));
    systemStatus(STATUS_ERROR);
  } else {
    sdCardFunctioning = true;
    if (SERIAL_DEBUG) Serial.println(F("SD Initialized"));
  }


  /* DS18B20 Initialization */
  dsSensors.begin();
  dsSensors.setResolution(12);


  /* ADS1115 Initialization */
  while ( !ads0.begin() ) {
    if (SERIAL_DEBUG) Serial.println(F("Failed to initialize ADS1115 0"));
    systemStatus(STATUS_ERROR);
  } if (SERIAL_DEBUG) Serial.println(F("ADS1115 0 Initialized"));
  
  while ( !ads1.begin(0x49) ) {
    if (SERIAL_DEBUG) Serial.println(F("Failed to initialize ADS1115 1"));
    systemStatus(STATUS_ERROR);
  } if (SERIAL_DEBUG) Serial.println(F("ADS1115 1 Initialized"));

  ads0.setGain(GAIN_ONE);
  ads0.setDataRate(RATE_ADS1115_8SPS);
  ads1.setGain(GAIN_ONE);
  ads1.setDataRate(RATE_ADS1115_8SPS);


  if (SERIAL_DEBUG) Serial.println(F("Setup Complete"));

}



/* Main Loop */
void loop() {

  // Wait for log time
  while (1) {
    currentTime = rtc.now();
    if (currentTime.unixtime() % LOG_INTERVAL_SECONDS == 0) break;
    delay(100);
  }

  // Collect data point
  char d[POINT_LENGTH_BYTES];
  logDataPoint(currentTime, d);
  if (SERIAL_DEBUG) Serial.println(F("Logging complete"));

  // Upload data if time or SD card has failed and log interval is > 2min - need to fix for stations to upload and collect at same time
  if ((currentTime.unixtime() % UPLOAD_INTERVAL_SECONDS == 0 || !sdCardFunctioning) ){

    if (SERIAL_DEBUG) Serial.println(F("Beginning Upload Process..."));
    
    cellRadioSetup();

    if (sdCardFunctioning) {
      
      // Read data to be logged
      logFile.open("tmp_buffer.txt", FILE_READ);
      
      char buff[111];
      unsigned long i = 0;
      while(logFile.available() > 0) {
        while (i < 110) {
          buff[i] = logFile.read();
          i++;
        }
        buff[i] = 0; // null terminator
        uploadData(buff);
        i = 0;
      }
      
      logFile.close();
      
      if (!sd.remove("tmp_buffer.txt")) { // Remove temp file
        if (SERIAL_DEBUG) Serial.println("SD File Removal Error!");
      }
      
    }
    
    else uploadData(d); // Single point if SD failed

    if (currentTime.unixtime() % CONFIG_INTERVAL_SECONDS == 0) {
      bool configResult = timeConfig();
      // config error checking
    }
    
  }


  // Sleep following operations
  systemSleep(SLEEP_INTERVAL_SECONDS);
  
  
}


/* Log data point */
void logDataPoint(DateTime currentTime, char *d) {

  systemStatus(STATUS_BLINK);

  // Turn on sensors with boot up delay
  digitalWrite(POWER_MOSFET_PIN, HIGH);
  delay(1000);

  uint32_t currentTimeStamp = currentTime.unixtime();

  // Temperatures
  dsSensors.requestTemperatures();
  for (int i = 0; i < 5; i++) {
    float tempC = dsSensors.getTempC(dsAddresses[i]);
    if (tempC != DEVICE_DISCONNECTED_C) {
      dtostrf(tempC, 6, 5, d + (7*i));
    } else {
      sprintf(d + (7*i), "%s", "-99.99");
    }
      d[6 + (7*i)] = ',';
  }

  // Moistures / voltages
  #define mstart 35
  for (int i = 0; i < 4; i++) { // ADS0
    dtostrf(
      1000*ads0.computeVolts(ads0.readADC_SingleEnded(i)),
      7, 5, d + mstart + (8*i));
    d[mstart+7 + (8*i)] = ',';
  }
  #define vstart 67
  for (int i = 0; i < 4; i++) { // ADS1
    dtostrf(
      1000*ads1.computeVolts(ads1.readADC_SingleEnded(i)),
      7, 5, d + vstart + (8*i));
    d[vstart+7 + (8*i)] = ',';
  }

  // Insert time
  sprintf(d+99, "%lu", currentTimeStamp);
  d[109] = ';';
  d[110] = 0;

  // Print
  if (SERIAL_DEBUG) {Serial.print("Logged: ");Serial.println(d);}

  // Log to SD card - new file each day to prevent loss - and upload buffer
  if (sdCardFunctioning) {
    char path[15];
    sprintf(path, "%04d_%02d_%02d.txt", currentTime.year(), currentTime.month(), currentTime.day());
    if (SERIAL_DEBUG) Serial.println(F("Starting save process..."));
    appendLogFile(path, d);
    appendLogFile("tmp_buffer.txt", d);
  }

  // Power off sensors
  digitalWrite(POWER_MOSFET_PIN, LOW);

  systemStatus(STATUS_SUCCESS);

  delay(100);
  
}



/* Upload data */
bool uploadData(char *d) {

  systemStatus(STATUS_BLINK);

  if (SERIAL_DEBUG) Serial.println(F("Uploading data..."));

  String dataString = "token=" + SERVER_API_KEY + "&id=" + STATION_ID + "&data=" + String(d);
  if (SERIAL_DEBUG) Serial.println(dataString);

  // Send data
  uploadComplete = false;
  assetTracker.sendHTTPPOSTdata(0, "/", "post_response.txt", dataString, SARA_R5_HTTP_CONTENT_APPLICATION_X_WWW);

  // Wait for upload to complete with 10 second timeout
  unsigned long start = millis();
  while (!uploadComplete) {
    assetTracker.poll();
    delay(10);
    if (millis() - start > 10000) {
      if (SERIAL_DEBUG) Serial.println(F("Failed to upload!"));
      systemStatus(STATUS_ERROR);
      return false;
    }
  }

  // Check for SUCCESS
  

  // Clear response file
  assetTracker.deleteFile("post_response.txt");

  if (SERIAL_DEBUG) Serial.println(F("Data uploaded"));
  systemStatus(STATUS_SUCCESS);

  return true;
  
}


/* Data upload complete */
void setUploadComplete(int profile, int command, int result) {
  uploadComplete = true;
}


/* Put the system into deepsleep */
void systemSleep(int sleepSeconds) {

  if (SERIAL_DEBUG) Serial.println("Starting sleep...");
  
  // Shut off radio on first boot
  if (!radioWasInitiallyTurnedOff) {
    assetTracker.begin(saraSerial, SARA_R5_DEFAULT_BAUD_RATE);
//    assetTracker.modulePowerOff();
    assetTracker.hardPowerOff();
    radioWasInitiallyTurnedOff = true;
  }

  // Shut down radio if it was on
  if (radioWasOn) {
    assetTracker.performPDPaction(0, SARA_R5_PSD_ACTION_DEACTIVATE);
//    assetTracker.modulePowerOff();
    assetTracker.hardPowerOff();
  }

  // Shut down peripheral power
  disableMicroSDPower();
  disableIMUPower();
  disableSPIPins();

  // Debug
  if (SERIAL_DEBUG) {
    Serial.print("Sleeping for ");
    Serial.print(sleepSeconds);
    Serial.println(" seconds...");
  }
  Serial.flush();

  // Set sleep time and execute
  esp_sleep_enable_timer_wakeup(sleepSeconds * 1000000); // seconds to microseconds
  esp_deep_sleep_start();
  
}


/* Retrieve system configuration and current time */
bool timeConfig() {

  // Download config data * need to add station ID
  assetTracker.sendHTTPGET(0, "/config.php?id=" + STATION_ID, "config_response.txt");

  // Wait for download to complete with 10 second timeout
  uploadComplete = false;
  unsigned long start = millis();
  while (!uploadComplete) {
    assetTracker.poll();
    delay(10);
    if (millis() - start > 10000) {
      if (SERIAL_DEBUG) Serial.println(F("Failed to download!"));
      systemStatus(STATUS_ERROR);
      return false;
    }
  }

  // Retrieve data from SARA filesystem
  String configResponse;
  assetTracker.getFileContents("config_response.txt", &configResponse);
  if (SERIAL_DEBUG) Serial.println(F("Config Response: "));
  if (SERIAL_DEBUG) Serial.println(configResponse);

  // * add error check for bad response

  // Process response by filtering out HTTP header and splitting on commas
  int maxParts = 8;
  String parts[maxParts];
  int partCount = 0;
  int strIndex = 0;
  int minIndex = 0;
  String delimiter = ",";
  
  String filteredResponse = configResponse.substring(configResponse.lastIndexOf('\n')+1);

  while (strIndex < filteredResponse.length() && partCount < maxParts) {
    strIndex = filteredResponse.indexOf(delimiter, minIndex);
    if (strIndex == -1) {
      strIndex = filteredResponse.length();
    }
    parts[partCount] = filteredResponse.substring(minIndex, strIndex);
    partCount++;
    minIndex = strIndex + 1;
  }

  // Set log interval ... 300 if error
  if (parts[0].toInt() == 0) LOG_INTERVAL_SECONDS = 300;
  else LOG_INTERVAL_SECONDS = parts[0].toInt();

  // Set upload interval ... 10800 if error
  if (parts[1].toInt() == 0) UPLOAD_INTERVAL_SECONDS = 10800;
  else UPLOAD_INTERVAL_SECONDS = parts[1].toInt();

  // Set new time
  rtc.adjust(DateTime(
    parts[2].toInt(), parts[3].toInt(), parts[4].toInt(),
    parts[5].toInt(), parts[6].toInt(), parts[7].toInt())
    );

  DateTime checkTime = rtc.now();
  if (SERIAL_DEBUG) {Serial.print("Configured time: ");Serial.println(checkTime.unixtime());}
  if (SERIAL_DEBUG) {Serial.print("Configured log interval: ");Serial.println(LOG_INTERVAL_SECONDS);}
  if (SERIAL_DEBUG) {Serial.print("Configured upload interval: ");Serial.println(UPLOAD_INTERVAL_SECONDS);}

  assetTracker.deleteFile("config_response.txt");

  return true;
  
}



/* System status indicator */
void systemStatus(int sts) {

  if (SERIAL_DEBUG) {Serial.print("Status: ");Serial.println(sts);}

  switch (sts) {
    case 0:
      digitalWrite(LED_PIN, HIGH);delay(50);digitalWrite(LED_PIN, LOW);
      break;
    case 1:
      digitalWrite(LED_PIN, HIGH);delay(50);digitalWrite(LED_PIN, LOW);delay(100);
      digitalWrite(LED_PIN, HIGH);delay(50);digitalWrite(LED_PIN, LOW);delay(100);
      digitalWrite(LED_PIN, HIGH);delay(50);digitalWrite(LED_PIN, LOW);
      break;
    case 2:
      digitalWrite(LED_PIN, HIGH);delay(500);digitalWrite(LED_PIN, LOW);delay(500);
    default:
      break;
  }
  
}



/* Cell Radio Setup */
bool cellRadioSetup() {

  if (SERIAL_DEBUG) Serial.println("Beginning to set up cell radio...");

  unsigned int startTime = millis();

  // Power on and connect
  assetTracker.invertPowerPin(true);
  while (1) {
    if (assetTracker.begin(saraSerial, SARA_R5_DEFAULT_BAUD_RATE)) {
      if (SERIAL_DEBUG) Serial.println(F("SARA-R5 communicatng ... Connecting to operator..."));
      break;
    }
    if ((millis() - startTime) > CELL_CONNECT_TIMEOUT_MS) {
      if (SERIAL_DEBUG) Serial.println(F("Failed to connect to SARA-R5"));
      return false;
    }
    if (SERIAL_DEBUG) Serial.println("SARA-R5 Connecting...");
    delay(1000);
  }

  radioWasOn = true;
  
  assetTracker.setNetworkProfile(MNO_GLOBAL);

  // Wait for radio to connect to an operator
  String currentOperator = "";
  while (assetTracker.getOperator(&currentOperator) != SARA_R5_SUCCESS) {
    if (SERIAL_DEBUG) {
      Serial.println(F("No operator..."));
      Serial.print(F("Connect time: "));
      Serial.print((millis() - startTime)/1000); Serial.println(F(" seconds"));
    }
    if ((millis() - startTime) > CELL_CONNECT_TIMEOUT_MS) {
      if (SERIAL_DEBUG) Serial.println(F("Failed to connect to an operator"));
      return false;
    }
    delay(5000);
  } if (SERIAL_DEBUG) {Serial.print(F("Connected to: "));Serial.println(currentOperator);}

  // Deactivate the PSD profile - in case one is already active
  if (assetTracker.performPDPaction(0, SARA_R5_PSD_ACTION_DEACTIVATE) != SARA_R5_SUCCESS) {
    if (SERIAL_DEBUG) Serial.println("No PSD profile was active...");
  } if (SERIAL_DEBUG) Serial.println("Setting PID...");

  // Map PSD profile 0 to the selected CID
  if (assetTracker.setPDPconfiguration(0, SARA_R5_PSD_CONFIG_PARAM_MAP_TO_CID, 1) != SARA_R5_SUCCESS) {
    if (SERIAL_DEBUG) Serial.println(F("Could not set PID..."));
    return false;
  } if (SERIAL_DEBUG) Serial.println(F("Setting protocol..."));

  // Set the protocol type - this needs to match the defined IP type for the CID (as opposed to what was granted by the network)
  if (assetTracker.setPDPconfiguration(0, SARA_R5_PSD_CONFIG_PARAM_PROTOCOL, SARA_R5_PSD_PROTOCOL_IPV4V6_V4_PREF) != SARA_R5_SUCCESS) {
    if (SERIAL_DEBUG) Serial.println(F("setPDPconfiguration (set protocol type) failed!"));
    return false;
  } 

  // Activate the PSD profile
  if (assetTracker.performPDPaction(0, SARA_R5_PSD_ACTION_ACTIVATE) != SARA_R5_SUCCESS) {
    if (SERIAL_DEBUG) Serial.println(F("performPDPaction (activate profile) failed! Freezing..."));
    return false;
  }

  // HTTP parameters
  assetTracker.resetHTTPprofile(0);
  assetTracker.setHTTPserverName(0, SERVER_URL);
  assetTracker.setHTTPserverPort(0, SERVER_PORT);
  assetTracker.setHTTPsecure(0, false); // some bug here with TLS versions. must be insecure
  assetTracker.setHTTPCommandCallback(&setUploadComplete);
  
  if (SERIAL_DEBUG) Serial.println("Cell radio ready!");
  
  return true;
  
}



void appendLogFile(char *path, char *d) {

  if (SERIAL_DEBUG) Serial.println(F("Saving ..."));
  
  // Open file
  if (!logFile.open(path, FILE_WRITE)) {
    if (SERIAL_DEBUG) Serial.println(F("Failed to open file"));
    sdCardFunctioning = false;
    systemStatus(STATUS_ERROR);
    return;
  } else if (SERIAL_DEBUG) Serial.println(F("Opened log file"));

  // Append to file
  if (!logFile.print(d)) {
    sdCardFunctioning = false;
    if (SERIAL_DEBUG) Serial.println(F("Failed write data to SD card"));
    systemStatus(STATUS_ERROR);
  } else if (SERIAL_DEBUG) Serial.println(F("Wrote data"));

  // Check for failure
  if (!logFile.close()) {
    sdCardFunctioning = false;
    if (SERIAL_DEBUG) Serial.println(F("Failed to close file"));
    systemStatus(STATUS_ERROR);
  } else if (SERIAL_DEBUG) Serial.println(F("Closed log file"));

  if (SERIAL_DEBUG) {Serial.print(F("Wrote data: "));Serial.println(d);}
  
}
