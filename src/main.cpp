#include <Arduino.h>
#include <bq769x0.h>
#include <Wire.h>
#include <registers.h>
#include <function.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <FastLED.h>
#include <OneButton.h>

#define BQ769x0 1

#define RXD2 16
#define TXD2 17
#define LED_PIN     19
#define NUM_LEDS    8

#define BMS_ALERT_PIN 34     // attached to interrupt INT0
#define BMS_BOOT_PIN 21      // connected to TS1 input
#define BMS_I2C_ADDRESS 0x08

#define EEPROM_FRAME_NAME_ADDRESS 0x00 //address 0 (32 characters), reserved 1 slot for null terminator
#define EEPROM_CONFIGURED_FLAG 0x20 //address 32

DynamicJsonDocument docBattery(768);
CRGB leds[NUM_LEDS];
bq769x0 BMS[3] = {bq769x0(bq76940, BMS_I2C_ADDRESS, 0), bq769x0(bq76940, BMS_I2C_ADDRESS, 1), bq769x0(bq76940, BMS_I2C_ADDRESS, 2)};
bool isFirstRun = true;
unsigned long currTime;
TwoWire wire = TwoWire(0);

const int addr = 23;
const int tombol = 18;
const int restartPin = 22;
OneButton addrButton(addr, false, false);
OneButton doorButton(tombol, false, false);
// OneButton restartButton(restartPin, true, true);
bool isAddrPressed = false;
bool isDoorPressed = false;
bool isRestartPressed = false;

const String firmwareVersion = "v0.1";
const String productionCode = "CMS-32-01";
const String chipType = "BQ769x0";

int sda = 26;
int scl = 27;

int menu = 1;

int BIDfromEhub;
int BID0 = 999;
int BID = 0 ;

unsigned long lastUpdateTime = 0;
unsigned long lastRefreshTime = 0;
bool isIdle = false;
bool isBrightnessDown = false;
int brightness = 0;
int buttonState;
int lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

int R;
int g;
int b;


String CMSFrameName = "undefined";

String serialIn = "";

void TCA9548A(uint8_t bus) {
  wire.beginTransmission(0x70);  // TCA9548A address
  wire.write(1 << bus);          // send byte to select bus
  wire.endTransmission();
}

void Scanner ()
{
  Serial.println ();
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;
  for (byte i = 8; i < 120; i++)
  {
    wire.beginTransmission (i);          // Begin I2C transmission Address (i)
    if (wire.endTransmission () == 0)  // Receive 0 = success (ACK response)
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);     // PCF8574 7 bit address
      Serial.println (")");
      count++;
    }
  }
  Serial.print ("Found ");
  Serial.print (count, DEC);        // numbers of devices
  Serial.println (" device(s).");
}


void sendInfo(int bid)
{
  String stringOut;
  DynamicJsonDocument doc(256);
  // doc["bid"] = bid;
  // doc["p_code"] = productionCode;
  // doc["ver"] = firmwareVersion;
  // doc["chip"] = chipType;
  doc["frame_name"] = CMSFrameName;
  doc["bid"] = bid;
  doc["p_code"] = productionCode;
  doc["ver"] = firmwareVersion;
  doc["chip"] = chipType;
  serializeJson(doc, stringOut);
  Serial2.print(stringOut);
  Serial2.print('\n');
}

void sendFrameInfo(int bid)
{
  String output;
  DynamicJsonDocument doc(96);
  doc["bid"] = bid;
  doc["frame_write"] = 1;
  doc["status"] = 1;
  serializeJson(doc, output);
  Serial2.print(output);
  Serial2.print('\n');
}

void OffBalancing(int x)
{

  DynamicJsonDocument docBattery(768);
  docBattery["BID"] = BID;
  serializeJson(docBattery, Serial2);
}

void GV()
{
  String output;
  BMS[0].update();
  BMS[1].update();
  BMS[2].update();
  DynamicJsonDocument docBattery(768);
  docBattery["BID"] = BID;
  JsonArray data = docBattery.createNestedArray("VCELL");
  for (int i = 1; i < 16; i ++)
  {
    //    DynamicJsonDocument docBattery(768);
    data.add(BMS[0].getCellVoltage(i));
    //    docBattery["C" + String(i)] = BMS[0].getCellVoltage(i);

  }
  for (int i = 1; i < 16; i ++)
  {
    int a = i + 15;
    data.add(BMS[1].getCellVoltage(i));
    //    DynamicJsonDocument docBattery(768);
    //    docBattery["C" + String(a)] = BMS[1].getCellVoltage(i);

  }
  for (int i = 1; i < 16; i ++)
  {
    int a = i + 30;
    data.add(BMS[2].getCellVoltage(i));
    //    DynamicJsonDocument docBattery(768);
    //    docBattery["C" + String(a)] = BMS[2].getCellVoltage(i);

  }
//  Serial.println();
//  serializeJson(docBattery, Serial2);
  serializeJson(docBattery, output);
  Serial2.print(output);
  Serial2.print('\n');
}

void GetVoltage ()
{
  DynamicJsonDocument docBattery(768);
  docBattery["BID"] = BID;
  serializeJson(docBattery, Serial2);
  for (int i = 1; i < 16; i ++)
  {
    DynamicJsonDocument docBattery(768);
    docBattery["C" + String(i)] = BMS[0].getCellVoltage(i);
    serializeJson(docBattery, Serial2);
  }
  for (int i = 1; i < 16; i ++)
  {
    int a = i + 15;
    DynamicJsonDocument docBattery(768);
    docBattery["C" + String(a)] = BMS[1].getCellVoltage(i);
    serializeJson(docBattery, Serial2);
  }
  for (int i = 1; i < 16; i ++)
  {
    int a = i + 30;
    DynamicJsonDocument docBattery(768);
    docBattery["C" + String(a)] = BMS[2].getCellVoltage(i);
    serializeJson(docBattery, Serial2);
  }
  Serial2.println(".");
}

void GetTemp()
{
  BMS[0].update();
  BMS[1].update();
  BMS[2].update();
  DynamicJsonDocument docBattery(768);
  docBattery["BID"] = BID;
  JsonArray data = docBattery.createNestedArray("TEMP");
  //  serializeJson(docBattery, Serial2);
  for (int j = 0; j < 3; j ++) {
    for (int i = 1; i < 4; i ++) {
      int a = j + 1;
      int b = i ;
//      DynamicJsonDocument docBattery(768);
      data.add(BMS[j].getTemperatureDegC(i));
      //      docBattery["BQ" + String(a) + "T" + String(b)] = BMS[j].getTemperatureDegC(i);

    }
  }
  String output;
//  serializeJson(docBattery, Serial2);
  serializeJson(docBattery, output);
  Serial2.print(output);
  Serial2.print('\n');


}

void getBQStatus(int bid)
{
  StaticJsonDocument<64> doc;
  int status = 1;
  doc["BID"] = bid;
  for (size_t i = 0; i < 3; i++)
  {
    if (BMS[i].isDeviceSleep())
    {
      status = 0;
      break;
    }
  }
  doc["WAKE_STATUS"] = status;
  if(isDoorPressed)
  {
    doc["DOOR_STATUS"] = 1;
  }
  else
  {
    doc["DOOR_STATUS"] = 0;
  }
  String output;
  serializeJson(doc, output);
  Serial2.print(output);
  Serial2.print('\n');
}

void GetDeviceStatus()
{
  DynamicJsonDocument docBattery(768);
  docBattery["BID"] = BID;
  serializeJson(docBattery, Serial2);
  for (int j = 0; j < 3; j ++) {
    int a = BMS[j].isDeviceSleep();
    Serial2.println(".");
    DynamicJsonDocument docBattery(768);
    docBattery["BQ" + String (j) + "STAT"] = a;
    serializeJson(docBattery, Serial2);
  }
}

void bqShutdown(int bid)
{
  for (size_t i = 0; i < 3; i++)
  {
    BMS[i].shutdown();
  }
  getBQStatus(bid);
}

void bqShut(int a)
{
  DynamicJsonDocument docBattery(768);
  docBattery["BID"] = BID;
  serializeJson(docBattery, Serial2);
  int c = a - 1 ;
  BMS[c].shutdown();
  int b = BMS[c].isDeviceSleep();
  Serial2.println(".");


  docBattery["BQ" + String (c) + "STAT"] = b;
  serializeJson(docBattery, Serial2);
}

void bqWakeUp(int bid)
{
  for (size_t i = 0; i < 3; i++)
  {
    BMS[i].wake();
  }
  getBQStatus(bid);
}

void bqWake (int h)
{
  DynamicJsonDocument docBattery(768);
  docBattery["BID"] = BID;

  int c = h - 1;
  BMS[c].wake();
  int a = BMS[c].isDeviceSleep();
  Serial2.println(".");

  docBattery["BQ" + String (c) + "STAT"] = a;
  serializeJson(docBattery, Serial2);
}

void doBalancing(int cellPos, int switchState)
{
  if (cellPos < 5)
  {
    BMS[0].setBalanceSwitch(1, cellPos, switchState);
  }
  else if (cellPos >= 5 && cellPos < 10)
  {
    BMS[0].setBalanceSwitch(2, cellPos - 5, switchState);
  }
  else if (cellPos >= 10 && cellPos < 15)
  {
    BMS[0].setBalanceSwitch(3, cellPos - 10, switchState);
  }

  else if (cellPos >= 15 && cellPos < 20)
  {
    BMS[1].setBalanceSwitch(1, cellPos - 15, switchState);
  }
  else if (cellPos >= 20 && cellPos < 25)
  {
    BMS[1].setBalanceSwitch(2, cellPos - 20, switchState);
  }
  else if (cellPos >= 25 && cellPos < 30)
  {
    BMS[1].setBalanceSwitch(3, cellPos - 25, switchState);
  }

  else if (cellPos >= 30 && cellPos < 35)
  {
    BMS[2].setBalanceSwitch(1, cellPos - 30, switchState);
  }
  else if (cellPos >= 35 && cellPos < 40)
  {
    BMS[2].setBalanceSwitch(2, cellPos - 35, switchState);
  }
  else if (cellPos >= 40)
  {
    BMS[2].setBalanceSwitch(3, cellPos - 40, switchState);
  }
}

void SetBalancing(int x)
{
  DynamicJsonDocument docBattery(768);
  docBattery["BID"] = BID;
  serializeJson(docBattery, Serial2);
  //////////////cell 1 - 10
  if ( x < 11)
  {
    if (x < 4)
    { ////c1 c2 c3
      int a = x - 1;
      BMS[0].setBalanceSwitch(1, a , 1);
      BMS[0].enableBalancingProtection();
      BMS[0].updateBalanceSwitches();
    }
    if (x == 4)
    { ////// c4
      BMS[0].setBalanceSwitch(1, x , 1);
      BMS[0].enableBalancingProtection();
      BMS[0].updateBalanceSwitches();
    }
    if ( x < 7)
    { //// c5 c6
      int a = x - 5;
      BMS[0].setBalanceSwitch(2, a , 1);
      BMS[0].enableBalancingProtection();
      BMS[0].updateBalanceSwitches();
    }

    if ( x == 7)
    { ///c7
      BMS[0].setBalanceSwitch(2, 4 , 1);
      BMS[0].enableBalancingProtection();
      BMS[0].updateBalanceSwitches();
    }

    if (x < 10)
    { //// c8 c9
      int a = x - 8;
      BMS[0].setBalanceSwitch(3, a , 1);
      BMS[0].enableBalancingProtection();
      BMS[0].updateBalanceSwitches();
    }

    if (x == 10)
    { //// c10
      BMS[0].setBalanceSwitch(3, 4 , 1);
      BMS[0].enableBalancingProtection();
      BMS[0].updateBalanceSwitches();
    }
  }
  /////////////end of cell 1 - 10

  //////////////cell 11 - 20
  if ( x < 21)
  {
    if (x < 14)
    { ////c11 c12 c13
      int a = x - 11;
      BMS[1].setBalanceSwitch(1, a , 1);
      BMS[1].enableBalancingProtection();
      BMS[1].updateBalanceSwitches();
    }
    if (x == 14)
    { ////// c14
      BMS[1].setBalanceSwitch(1, 4 , 1);
      BMS[1].enableBalancingProtection();
      BMS[1].updateBalanceSwitches();
    }
    if ( x < 17)
    { //// c15 c16
      int a = x - 15;
      BMS[1].setBalanceSwitch(2, a , 1);
      BMS[1].enableBalancingProtection();
      BMS[1].updateBalanceSwitches();
    }

    if ( x == 17)
    { ///c17
      BMS[1].setBalanceSwitch(2, 4 , 1);
      BMS[1].enableBalancingProtection();
      BMS[1].updateBalanceSwitches();
    }

    if (x < 20)
    { //// c18 c19
      int a = x - 18;
      BMS[1].setBalanceSwitch(3, a , 1);
      BMS[1].enableBalancingProtection();
      BMS[1].updateBalanceSwitches();
    }

    if (x == 20)
    { //// c10
      BMS[1].setBalanceSwitch(3, 4 , 1);
      BMS[1].enableBalancingProtection();
      BMS[1].updateBalanceSwitches();
    }
  }
  /////end of cell 11- 20

  //////cell 21 - 32

  if (x < 33 )
  {
    if (x < 24)
    {
      int a = x - 21;
      BMS[2].setBalanceSwitch(1, a , 1);
      BMS[2].enableBalancingProtection();
      BMS[2].updateBalanceSwitches();
    }

    if ( x == 24)
    {
      BMS[2].setBalanceSwitch(1, 4 , 1);
      BMS[2].enableBalancingProtection();
      BMS[2].updateBalanceSwitches();
    }

    if ( x < 28 )
    {
      int a = x - 25;
      BMS[2].setBalanceSwitch(2, a , 1);
      BMS[2].enableBalancingProtection();
      BMS[2].updateBalanceSwitches();
    }

    if ( x == 28)
    {
      BMS[2].setBalanceSwitch(2, 4 , 1);
      BMS[2].enableBalancingProtection();
      BMS[2].updateBalanceSwitches();
    }

    if ( x < 32)
    {
      int a = x - 29;
      BMS[2].setBalanceSwitch(3, a , 1);
      BMS[2].enableBalancingProtection();
      BMS[2].updateBalanceSwitches();
    }

    if ( x == 32 )
    {
      BMS[2].setBalanceSwitch(3, 4 , 1);
      BMS[2].enableBalancingProtection();
      BMS[2].updateBalanceSwitches();
    }

  }
  ////////end of cell 21 - 32

  docBattery["RBAL1.1"] = BMS[0].readReg(CELLBAL1);
  docBattery["RBAL1.2"] = BMS[0].readReg(CELLBAL2);
  docBattery["RBAL1.3"] = BMS[0].readReg(CELLBAL3);
  Serial2.println(".");
  docBattery["RBAL2.1"] = BMS[1].readReg(CELLBAL1);
  docBattery["RBAL2.2"] = BMS[1].readReg(CELLBAL2);
  docBattery["RBAL2.3"] = BMS[1].readReg(CELLBAL3);
  Serial2.println(".");
  docBattery["RBAL3.1"] = BMS[2].readReg(CELLBAL1);
  docBattery["RBAL3.2"] = BMS[2].readReg(CELLBAL2);
  docBattery["RBAL3.3"] = BMS[2].readReg(CELLBAL3);
  Serial2.println(".");
  serializeJson(docBattery, Serial2);
  Serial2.println(".");
}

void readBalancingRequest(int bid)
{
  DynamicJsonDocument doc(768);
  doc["BID"] = bid;
  doc["RBAL1.1"] = BMS[0].readReg(CELLBAL1);
  doc["RBAL1.2"] = BMS[0].readReg(CELLBAL2);
  doc["RBAL1.3"] = BMS[0].readReg(CELLBAL3);
  doc["RBAL2.1"] = BMS[1].readReg(CELLBAL1);
  doc["RBAL2.2"] = BMS[1].readReg(CELLBAL2);
  doc["RBAL2.3"] = BMS[1].readReg(CELLBAL3);
  doc["RBAL3.1"] = BMS[2].readReg(CELLBAL1);
  doc["RBAL3.2"] = BMS[2].readReg(CELLBAL2);
  doc["RBAL3.3"] = BMS[2].readReg(CELLBAL3);
  String output;
  serializeJson(doc, output);
  Serial2.print(output);
  Serial2.print('\n');
}

void ReadBalancing()
{
  DynamicJsonDocument docBattery(768);
  docBattery["BID"] = BID;
  serializeJson(docBattery, Serial2);
  docBattery["RBAL1.1"] = BMS[0].readReg(CELLBAL1);
  docBattery["RBAL1.2"] = BMS[0].readReg(CELLBAL2);
  docBattery["RBAL1.3"] = BMS[0].readReg(CELLBAL3);
  Serial2.println(".");
  docBattery["RBAL2.1"] = BMS[1].readReg(CELLBAL1);
  docBattery["RBAL2.2"] = BMS[1].readReg(CELLBAL2);
  docBattery["RBAL2.3"] = BMS[1].readReg(CELLBAL3);
  Serial2.println(".");
  docBattery["RBAL3.1"] = BMS[2].readReg(CELLBAL1);
  docBattery["RBAL3.2"] = BMS[2].readReg(CELLBAL2);
  docBattery["RBAL3.3"] = BMS[2].readReg(CELLBAL3);
  Serial2.println(".");
  serializeJson(docBattery, Serial2);
  Serial2.println(".");
}

void clearBalancingRequest(int bid)
{
  BMS[0].clearBalanceSwitches();
  BMS[1].clearBalanceSwitches();
  BMS[2].clearBalanceSwitches();
  BMS[0].updateBalanceSwitches();
  BMS[1].updateBalanceSwitches();
  BMS[2].updateBalanceSwitches();
  readBalancingRequest(bid);
}

void ClearBalancing()
{
  DynamicJsonDocument docBattery(768);
  docBattery["BID"] = BID;
  serializeJson(docBattery, Serial2);
  BMS[0].clearBalanceSwitches();
  BMS[1].clearBalanceSwitches();
  BMS[2].clearBalanceSwitches();
  BMS[0].updateBalanceSwitches();
  BMS[1].updateBalanceSwitches();
  BMS[2].updateBalanceSwitches();
  docBattery["CBAL"] = "OK";
  serializeJson(docBattery, Serial2);
  Serial2.println(".");
}
void GetVpack() {
  DynamicJsonDocument docBattery(768);
  JsonArray data = docBattery.createNestedArray("VPACK");
  int b = BMS[0].getBatteryVoltage() + BMS[1].getBatteryVoltage() + BMS[2].getBatteryVoltage();
  docBattery["BID"] = BID;
  data.add(b);
  for (int i = 0; i < 3; i ++) {
    int a = i + 1;
    data.add(BMS[i].getBatteryVoltage());
    //    docBattery["BQ" + String(a)] = BMS[i].getBatteryVoltage();
    //    serializeJson(docBattery, Serial2);
  }
  String output;
//  serializeJson(docBattery, Serial2);
  serializeJson(docBattery, output);
  Serial2.print(output);
  Serial2.print('\n');
}

void cmsRestart()
{
  for (size_t i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
  delay(500);
  ESP.restart();
}

void addrButtonClick()
{
  // Serial2.print("Long Press Start Detected");
  // Serial2.print('\n');
  // leds[0] = CRGB::Yellow;
  // FastLED.show();
  isAddrPressed = true;
}

void addrButtonLongPressStart()
{
  // Serial2.print("Long Press Start Detected");
  // Serial2.print('\n');
  // leds[0] = CRGB::Yellow;
  // FastLED.show();
  // isAddrPressed = true;
}

void addrButtonLongPressStop()
{
  // Serial2.print("Long Press Stop Detected");
  // Serial2.print('\n');
  // leds[0] = CRGB::Black;
  // FastLED.show();
  // isAddrPressed = false;
}

void doorButtonLongPressStart()
{
  isDoorPressed = true;
}

void doorButtonLongPressStop()
{
  isDoorPressed = false;
}

void restartButtonClick()
{
  // Serial2.println("Restart");
  for (size_t i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
  delay(500);
  ESP.restart();
}

void setup() {
  // put your setup code here, to run once:
  EEPROM.begin(128);

  if(EEPROM.read(EEPROM_CONFIGURED_FLAG) == 1)
  {
    CMSFrameName = EEPROM.readString(EEPROM_FRAME_NAME_ADDRESS);
  }

  Serial.begin(9600);
  Serial2.begin(115200);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  wire.setPins(sda, scl);
  wire.begin();

  addrButton.setPressTicks(200);
  addrButton.attachClick(addrButtonClick);
  addrButton.attachLongPressStart(addrButtonLongPressStart);
  addrButton.attachLongPressStop(addrButtonLongPressStop);
  doorButton.attachLongPressStart(doorButtonLongPressStart);
  doorButton.attachLongPressStop(doorButtonLongPressStop);
  // restartButton.setDebounceTicks(100);
  // restartButton.attachClick(restartButtonClick);

  for (int i = 0; i < 3; i ++)
  {
    BMS[i].setI2C(&wire);
  }

  // Scanner();
  int err;
  int lastErr;
  for (int i = 0; i < 3; i ++)
  {

    err = BMS[i].begin(BMS_ALERT_PIN, BMS_BOOT_PIN, &TCA9548A);
    lastErr = err;
    err = lastErr | err;
  }

  if (err)
  {
    Serial.println("BMS Init Error");
  }
  BMS[0].setCellConfiguration(BMS[0].CELL_10);
  BMS[1].setCellConfiguration(BMS[1].CELL_10);
  BMS[2].setCellConfiguration(BMS[2].CELL_12);

  // BMS.setTemperatureLimits(-20, 45, 0, 45);
  // BMS.setShuntResistorValue(5);
  // BMS.setShortCircuitProtection(14000, 200);  // delay in us
  // BMS.setOvercurrentChargeProtection(8000, 200);  // delay in ms
  // BMS.setOvercurrentDischargeProtection(8000, 320); // delay in ms
  // BMS.setCellUndervoltageProtection(2600, 2); // delay in s
  // BMS.setCellOvervoltageProtection(3650, 2);  // delay in s

  // BMS.setBalancingThresholds(0, 3300, 20);  // minIdleTime_min, minCellV_mV, maxVoltageDiff_mV
  // BMS.setIdleCurrentThreshold(100);
  // BMS.enableAutoBalancing();
  // BMS.enableDischarging();

  /*
  for (int i = 0; i < 3; i++)
  {
    Serial.println("BMS " + String(i + 1) + " Configuration");
    int data = BMS[i].readReg(SYS_STAT);
    Serial.print("SYS_STAT " + String(i + 1) + " : ");
    Serial.println(data, BIN);
    Serial.println("Clearing SYS_STAT " + String(i + 1));
    BMS[i].writeReg(SYS_STAT, data);
    data = BMS[i].readReg(SYS_STAT);
    Serial.print("SYS_STAT " + String(i + 1) + " : ");
    Serial.println(data, BIN);
  }
  */
  delay(1000);
  leds[0] = CRGB::Orange;
  FastLED.setBrightness(20);
  FastLED.show();
}

void loop() {
  // put your main code here, to run repeatedly:
  bool isJsonCompleted = false;
  addrButton.tick();
  doorButton.tick();
  // restartButton.tick();
  // Serial.println("Multiple BMS Example");
  // DynamicJsonDocument docBattery(768);
  // deserializeJson(docBattery, Serial2);
  // JsonObject object = docBattery.as<JsonObject>();
  ///////////////////////////nyalain bms
  if (isFirstRun)
  {
    for (int i = 0; i < 3; i ++)
    {
      BMS[i].update();
    }
    currTime = millis();
    isFirstRun = false;
  }

  ///////////////////////////end of nyalain

  if ((millis() - currTime) > 250 )
  {
    for (int i = 0; i < 3; i ++)
    {
      BMS[i].update();
    }
    currTime = millis();
  }

  ////////////////////////////////////////////////////////////////
  if (menu == 1)
  {
    leds[0] = CRGB::Orange;
    if (isAddrPressed) 
    {
      for (size_t i = 0; i < NUM_LEDS; i++)
      {
        leds[i] = CRGB(0,0,0);
      }
      StaticJsonDocument<128> doc;
      String output;
      doc["BID_STATUS"] = 1;
      // serializeJson(docBattery, Serial2);
      serializeJson(doc, output);
      Serial2.print(output);
      Serial2.print('\n');
      // Serial2.println("GETID");
      delay(20);
      // isAddrPressed = false;
      menu = 2;
    }
    else
    {
      while (Serial2.available())
      {
        char in = Serial2.read();
        if (in != '\n')
        {
          serialIn += in;
        }
        else
        {
          isJsonCompleted = true;
          break;
        }
      }
      if(isJsonCompleted)
      {
        StaticJsonDocument<128> doc;
        deserializeJson(doc, serialIn);
        if(doc.containsKey("RESTART"))
        {
          int bid = doc["BID"];
          int restart = doc["RESTART"];
          if(bid == 255)
          {
            if(restart)
            {
              cmsRestart();
            }
          }
        }
        serialIn = "";
      }
    }
    // delay(10);
  }

  if (menu == 2)
  {
    int timeout = 0;
    bool isJsonCompleted = false;
    bool isRetry = true;
    String serialIn = "";
    if(isAddrPressed)
    {
      while (isRetry)
      {
        if (timeout > 50)
        {
          menu = 1;
          isAddrPressed = false;
          break;
        }
        while (Serial2.available())
        {
          char in = Serial2.read();
          if (in != '\n')
          {
            serialIn += in;
          }
          else
          {
            isJsonCompleted = true;
            break;
          }
        }
        
        if(isJsonCompleted)
        {
          StaticJsonDocument<128> doc;
          deserializeJson(doc, serialIn);
          if(doc.containsKey("BID_ADDRESS"))
          {
            BID = doc["BID_ADDRESS"];
            if(BID > 0)
            {
              StaticJsonDocument<128> docBat;
              String output;
              docBat["BID"] = BID;
              docBat["RESPONSE"] = 1;
              serializeJson(docBat, output);
              Serial2.print(output);
              Serial2.print('\n');
              delay(20);
              int no = BID - 1;
              leds[no] = CRGB::LightSeaGreen;
              menu = 3;
              isJsonCompleted = false;
              isRetry = false;
            }
          }
          serialIn = "";
        }
        timeout++;
        delay(10);
      }
      isAddrPressed = false;
    }
    else
    {
      while (Serial2.available())
      {
        char in = Serial2.read();
        if (in != '\n')
        {
          serialIn += in;
        }
        else
        {
          isJsonCompleted = true;
          break;
        }
      }
      if(isJsonCompleted)
      {
        StaticJsonDocument<128> doc;
        deserializeJson(doc, serialIn);
        if(doc.containsKey("RESTART"))
        {
          int bid = doc["BID"];
          int restart = doc["RESTART"];
          if(bid == 255)
          {
            if(restart)
            {
              cmsRestart();
            }
          }
        }
        serialIn = "";
      }
    }
    
  }

  if (menu == 3)
  {
    while (Serial2.available())
    {
      char in = Serial2.read();
      if (in != '\n')
      {
        serialIn += in;
      }
      else
      {
        isJsonCompleted = true;
        break;
      }
    }
    
    if (isJsonCompleted)
    {
      isJsonCompleted = false;
      DynamicJsonDocument docBattery(1024);
      deserializeJson(docBattery, serialIn);
      serialIn = "";
      JsonObject object = docBattery.as<JsonObject>();
      BIDfromEhub = docBattery["BID"];
      int vcell = 0;
      int ReadBal = 0;
      int SetBal = 0;
      int OffBal = 0;
      int ClearBal = 0;
      int setled = 0;
      int numled = 0;
      int rjson = 0;
      int gjson = 0;
      int bjson = 0;
      int vpack = 0;
      int temp = 0;
      int rbq = 0;
      int sbq = 0;
      int wbq = 0;
      int info = 0;
      int frameWrite = 0;
      int restart = 0;
      vcell = docBattery["VCELL"];
      ReadBal = docBattery["RBAL"];
      SetBal = docBattery["SBAL"];
      OffBal = docBattery["OBAL"];
      ClearBal = docBattery["CBAL"];
      setled = docBattery["LEDSET"];
      numled = docBattery["L"];
      rjson = docBattery["R"];
      gjson = docBattery["G"];
      bjson = docBattery["B"];
      vpack = docBattery ["VPACK"];
      temp = docBattery ["TEMP"];
      rbq  = docBattery ["RBQ"];
      sbq = docBattery ["SBQ"];
      wbq = docBattery ["WBQ"];
      info = docBattery ["INFO"];
      frameWrite = docBattery ["frame_write"];
      restart = docBattery["RESTART"];
      
      //    if (digitalRead(tombol) == HIGH) {
      //      Serial2.println("HIGH");
      //      delay(100);
      //    }
      //    if (digitalRead(tombol) == LOW) {
      //      Serial2.println("LOW");
      //      delay(100);
      //    }
      if (BIDfromEhub == BID && vcell == 1 ) {
        //      GetVoltage();
        GV();
        lastUpdateTime = millis();
      }

      if (BIDfromEhub == BID && vpack == 1)
      {
        GetVpack();
        //      GetDeviceStatus();
        lastUpdateTime = millis();
      }

      if (BIDfromEhub == BID && temp == 1)
      {
        GetTemp();
        lastUpdateTime = millis();
      }
      
      if ( BIDfromEhub == BID && ReadBal == 1 ) {
        readBalancingRequest(BIDfromEhub);
        lastUpdateTime = millis();
      }

      if (BIDfromEhub == BID && SetBal == 1 ) 
      {
        if (docBattery.containsKey("cball"))
        {
          JsonArray cball = docBattery["cball"];
          int arrSize = cball.size();
          if (arrSize >= 45)
          {
            for (size_t i = 0; i < arrSize ; i++)
            {
              int switchState = cball[i];
              doBalancing(i, switchState);
            }
            for (size_t i = 0; i < 3; i++)
            {
              BMS[i].updateBalanceSwitches();
            }
            DynamicJsonDocument doc(768);
            doc["BID"] = BID;
            doc["RBAL1.1"] = BMS[0].readReg(CELLBAL1);
            doc["RBAL1.2"] = BMS[0].readReg(CELLBAL2);
            doc["RBAL1.3"] = BMS[0].readReg(CELLBAL3);
            doc["RBAL2.1"] = BMS[1].readReg(CELLBAL1);
            doc["RBAL2.2"] = BMS[1].readReg(CELLBAL2);
            doc["RBAL2.3"] = BMS[1].readReg(CELLBAL3);
            doc["RBAL3.1"] = BMS[2].readReg(CELLBAL1);
            doc["RBAL3.2"] = BMS[2].readReg(CELLBAL2);
            doc["RBAL3.3"] = BMS[2].readReg(CELLBAL3);
            String output;
            serializeJson(doc, output);
            Serial2.print(output);
            Serial2.print('\n');
          }
        }
        lastUpdateTime = millis();
      }

      if (BIDfromEhub == BID && OffBal == 1)
      {
        OffBalancing(OffBal);
        lastUpdateTime = millis();
      }

      if (BIDfromEhub == BID && ClearBal == 1 ) {
        clearBalancingRequest(BIDfromEhub);
        lastUpdateTime = millis();
      }

      if ( BIDfromEhub == BID && setled == 1 )
      {
        int status = 0;
        int numOfLed = docBattery["NUM_OF_LED"];
        JsonArray led_rgb = docBattery["LED_RGB"];
        if(numOfLed > 0)
        {
          for (size_t i = 0; i < numOfLed; i++)
          {
            JsonArray rgbValue = led_rgb[i];
            leds[i] = CRGB(rgbValue[0], rgbValue[1], rgbValue[2]);
          }
          status = 1;
        }
        DynamicJsonDocument doc(64);
        doc["BID"] = BID;
        doc["LEDSET"] = 1;
        doc["STATUS"] = status;
        String output;
        serializeJson(doc, output);
        Serial2.print(output);
        Serial2.print('\n');
        lastUpdateTime = millis();
      }

      if (BIDfromEhub == BID && info == 1 ) {
        // String output;
        sendInfo(BIDfromEhub);
        // Serial2.print(output);
        // Serial2.print('\n');
        lastUpdateTime = millis();
      } 

      if (BIDfromEhub == BID && frameWrite == 1 ) {
        // String output;
        // Serial2.println("Frame Write Processing");
        String newCMSFrameName = docBattery["frame_name"].as<String>();
        if(newCMSFrameName.length() < 31) // reserved 1 character for null terminator
        {
          if(newCMSFrameName != CMSFrameName)
          {
            EEPROM.writeString(EEPROM_FRAME_NAME_ADDRESS, CMSFrameName);
            EEPROM.write(EEPROM_CONFIGURED_FLAG, 1);
            sendFrameInfo(BIDfromEhub);
            EEPROM.commit();
          }
        }
        lastUpdateTime = millis();
        // Serial2.print(output);
        // Serial2.print('\n');
      } 

      if (BIDfromEhub == BID && rbq == 1)
      {
        // GetDeviceStatus();
        getBQStatus(BIDfromEhub);
        lastUpdateTime = millis();
      }

      if (BIDfromEhub == BID && sbq == 1)
      {
        bqShutdown(BIDfromEhub);
        lastUpdateTime = millis();
        // bqShut(sbq);
      }

      if ((BIDfromEhub == BID || BIDfromEhub == 255) && restart == 1)
      {
        lastUpdateTime = millis();
        cmsRestart();
        // bqShut(sbq);
      }

      if (BIDfromEhub == BID && wbq == 1)
      {
        bqWakeUp(BIDfromEhub);
        lastUpdateTime = millis();
        // bqWake(wbq);
      }
    }
    
  }
  
  if (menu == 4)
  {
    Serial.println("minta No id ulang");
    DynamicJsonDocument docBattery(768);
    docBattery["BID"] = "noBID";
    String output;
    // serializeJson(docBattery, Serial2);
    serializeJson(docBattery, output);
    Serial2.print(output);
    Serial2.print('\n');
    delay(100);
    menu = 1;
  }
  FastLED.setBrightness(20);
  FastLED.show();

}
