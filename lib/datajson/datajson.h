#include "bq769x0.h"

#include <ArduinoJson.h>
#include <Arduino.h>
#include <Wire.h>
#include "DataDef.h"


void GetVpack() {
  int b = BMS[0].getBatteryVoltage() + BMS[1].getBatteryVoltage() + BMS[2].getBatteryVoltage();
  DynamicJsonDocument docBattery(768);
  docBattery["VPACK"] = b;
  serializeJson(docBattery, Serial2);
  Serial2.println(".");
  for (int i = 0; i < 3; i ++) {
    int a = i + 1;
    DynamicJsonDocument docBattery(768);
    docBattery["BQ" + String(a)] = BMS[i].getBatteryVoltage();
    serializeJson(docBattery, Serial2);
  }
}
