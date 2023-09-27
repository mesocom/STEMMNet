#include <SparkFun_u-blox_SARA-R5_Arduino_Library.h>
#include "AssetTrackerPins.h"
#include <DS18B20.h>

#include <OneWire.h>
#include <DallasTemperature.h>

OneWire oneWire(D0);
DallasTemperature dsSensors(&oneWire);

DeviceAddress dsAddresses[5] = {
  {0x28, 0x13, 0x7f, 0xd, 0xf, 0x0, 0x0, 0xdc},
  {0x28, 0xc1, 0x21, 0x64, 0xe, 0x0, 0x0, 0xe9},
  {0x28, 0x20, 0x8e, 0xf8, 0xc, 0x0, 0x0, 0x17},
  {0x28, 0x36, 0x11, 0xf8, 0xc, 0x0, 0x0, 0x26},
  {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}
};


SARA_R5 assetTracker(SARA_PWR);

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.print("Test");

  initializeAssetTrackerPins();
  pinMode(PWM0, OUTPUT);
  
  digitalWrite(PWM0, HIGH);

}

void loop() {

//  dsSensors.requestTemperatures();
//
//  for (int i = 0; i < 5; i++) {
//    Serial.println(dsSensors.getTempC(dsAddresses[i]));
//  }
  
  OneWire ow(D0);

  uint8_t address[8];
  uint8_t count = 0;

  if (ow.search(address)) {
    
    do {
      count++;
    } while (ow.search(address));

    if (address[0] != 40) {return;}

    Serial.print("{");
    for (int i = 0; i < 7; i++) {Serial.print("0x");Serial.print(address[i], HEX);Serial.print(", ");}
    Serial.print("0x");Serial.print(address[7], HEX);
    Serial.println("},");
    
  }

  delay(1000);

}
