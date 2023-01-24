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
#define EEPROM_FRAME_ADDRESS_CONFIGURED_FLAG 0x20 //address 32
#define EEPROM_CMS_CODE_ADDRESS 0x30 //address 48 (32 characters), reserved 1 slot for null terminator
#define EEPROM_CMS_ADDRESS_CONFIGURED_FLAG 0x50 //address 80
#define EEPROM_BASE_CODE_ADDRESS 0x60 //address 96 (32 characters), reserved 1 slot for null terminator
#define EEPROM_BASE_ADDRESS_CONFIGURED_FLAG 0x80 //address 128
#define EEPROM_MCU_CODE_ADDRESS 0x70 //address 112 (32 characters), reserved 1 slot for null terminator
#define EEPROM_MCU_ADDRESS_CONFIGURED_FLAG 0x90 //address 144
#define EEPROM_SITE_LOCATION_ADDRESS 0xA0 //address 160 (32 characters), reserved 1 slot for null terminator
#define EEPROM_SITE_LOCATION_CONFIGURED_FLAG 0xC0 //address 192


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

const String firmwareVersion = "v1.0.1";
const String chipType = "bq769x0";
String CMSFrameName = "FRAME-32-NA";
String cmsCodeName = "CMS-32-NA";
String baseCodeName = "BASE-32-NA";
String mcuCodeName = "MCU-32-NA";
String siteLocationName = "SITE-32-NA";


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
  doc["frame_name"] = CMSFrameName;
  doc["bid"] = bid;
  doc["cms_code"] = cmsCodeName;
  doc["base_code"] = baseCodeName;
  doc["mcu_code"] = mcuCodeName;
  doc["site_location"] = siteLocationName;
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

void sendCMSCodeInfo(int bid)
{
  String output;
  DynamicJsonDocument doc(96);
  doc["bid"] = bid;
  doc["cms_write"] = 1;
  doc["status"] = 1;
  serializeJson(doc, output);
  Serial2.print(output);
  Serial2.print('\n');
}

void sendBaseCodeInfo(int bid)
{
  String output;
  DynamicJsonDocument doc(96);
  doc["bid"] = bid;
  doc["base_write"] = 1;
  doc["status"] = 1;
  serializeJson(doc, output);
  Serial2.print(output);
  Serial2.print('\n');
}

void sendMcuCodeInfo(int bid)
{
  String output;
  DynamicJsonDocument doc(96);
  doc["bid"] = bid;
  doc["mcu_write"] = 1;
  doc["status"] = 1;
  serializeJson(doc, output);
  Serial2.print(output);
  Serial2.print('\n');
}

void sendSiteLocationInfo(int bid)
{
  String output;
  DynamicJsonDocument doc(96);
  doc["bid"] = bid;
  doc["site_write"] = 1;
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
    data.add(BMS[0].getCellVoltage(i));
  }
  for (int i = 1; i < 16; i ++)
  {
    int a = i + 15;
    data.add(BMS[1].getCellVoltage(i));
  }
  for (int i = 1; i < 16; i ++)
  {
    int a = i + 30;
    data.add(BMS[2].getCellVoltage(i));
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
  for (int j = 0; j < 3; j ++) {
    for (int i = 1; i < 4; i ++) {
      int a = j + 1;
      int b = i ;
      data.add(BMS[j].getTemperatureDegC(i));

    }
  }
  String output;
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

void bqShutdown(int bid)
{
  for (size_t i = 0; i < 3; i++)
  {
    BMS[i].shutdown();
  }
  getBQStatus(bid);
}

void bqWakeUp(int bid)
{
  for (size_t i = 0; i < 3; i++)
  {
    BMS[i].wake();
  }
  getBQStatus(bid);
}

void setBalancing(int cellPos, int switchState)
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

void GetVpack() {
  DynamicJsonDocument docBattery(768);
  JsonArray data = docBattery.createNestedArray("VPACK");
  int b = BMS[0].getBatteryVoltage() + BMS[1].getBatteryVoltage() + BMS[2].getBatteryVoltage();
  docBattery["BID"] = BID;
  data.add(b);
  for (int i = 0; i < 3; i ++) {
    int a = i + 1;
    data.add(BMS[i].getBatteryVoltage());
  }
  String output;
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
  EEPROM.begin(256);

  if(EEPROM.read(EEPROM_FRAME_ADDRESS_CONFIGURED_FLAG) == 1)
  {
    CMSFrameName = EEPROM.readString(EEPROM_FRAME_NAME_ADDRESS);
  }

  if(EEPROM.read(EEPROM_CMS_ADDRESS_CONFIGURED_FLAG) == 1)
  {
    cmsCodeName = EEPROM.readString(EEPROM_CMS_CODE_ADDRESS);
  }

  if(EEPROM.read(EEPROM_BASE_ADDRESS_CONFIGURED_FLAG) == 1)
  {
    baseCodeName = EEPROM.readString(EEPROM_BASE_CODE_ADDRESS);
  }

  if(EEPROM.read(EEPROM_MCU_ADDRESS_CONFIGURED_FLAG) == 1)
  {
    mcuCodeName = EEPROM.readString(EEPROM_MCU_CODE_ADDRESS);
  }

  if(EEPROM.read(EEPROM_SITE_LOCATION_CONFIGURED_FLAG) == 1)
  {
    siteLocationName = EEPROM.readString(EEPROM_SITE_LOCATION_ADDRESS);
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

  if (isFirstRun)
  {
    for (int i = 0; i < 3; i ++)
    {
      BMS[i].update();
    }
    currTime = millis();
    isFirstRun = false;
  }


  if ((millis() - currTime) > 250 )
  {
    for (int i = 0; i < 3; i ++)
    {
      BMS[i].update();
    }
    currTime = millis();
  }

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
      serializeJson(doc, output);
      Serial2.print(output);
      Serial2.print('\n');
      delay(20);
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
      int cmsCodeWrite = 0;
      int baseCodeWrite = 0;
      int mcuCodeWrite = 0;
      int siteLocationWrite = 0;
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
      cmsCodeWrite = docBattery["cms_write"];
      baseCodeWrite = docBattery["base_write"];
      mcuCodeWrite = docBattery["mcu_write"];
      siteLocationWrite = docBattery["site_write"];
      restart = docBattery["RESTART"];
      
      if (BIDfromEhub == BID && vcell == 1 ) {
        GV();
        lastUpdateTime = millis();
      }

      if (BIDfromEhub == BID && vpack == 1)
      {
        GetVpack();
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
              bool state = false;
              if (switchState > 0)
              {
                state = true;
              }
              setBalancing(i, state);
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
        sendInfo(BIDfromEhub);
        lastUpdateTime = millis();
      } 

      if (BIDfromEhub == BID && frameWrite == 1 ) {
        String newCMSFrameName = docBattery["frame_name"].as<String>();
        if(newCMSFrameName.length() < 31) // reserved 1 character for null terminator
        {
          if(newCMSFrameName != CMSFrameName)
          {
            EEPROM.writeString(EEPROM_FRAME_NAME_ADDRESS, newCMSFrameName);
            EEPROM.write(EEPROM_FRAME_ADDRESS_CONFIGURED_FLAG, 1);
            EEPROM.commit();
            sendFrameInfo(BIDfromEhub);
            CMSFrameName = newCMSFrameName;
          }
        }
        lastUpdateTime = millis();
      }

      if (BIDfromEhub == BID && cmsCodeWrite == 1 ) {
        String newCMSCodeName = docBattery["cms_code"].as<String>();
        if(newCMSCodeName.length() < 31) // reserved 1 character for null terminator
        {
          if(newCMSCodeName != cmsCodeName)
          {
            EEPROM.writeString(EEPROM_CMS_CODE_ADDRESS, newCMSCodeName);
            EEPROM.write(EEPROM_CMS_ADDRESS_CONFIGURED_FLAG, 1);
            EEPROM.commit();
            sendCMSCodeInfo(BIDfromEhub);
            cmsCodeName = newCMSCodeName;
          }
        }
        lastUpdateTime = millis();
      }

      if (BIDfromEhub == BID && baseCodeWrite == 1 ) {
        String newBaseCodeName = docBattery["base_code"].as<String>();
        if(newBaseCodeName.length() < 31) // reserved 1 character for null terminator
        {
          if(newBaseCodeName != baseCodeName)
          {
            EEPROM.writeString(EEPROM_BASE_CODE_ADDRESS, newBaseCodeName);
            EEPROM.write(EEPROM_BASE_ADDRESS_CONFIGURED_FLAG, 1);
            EEPROM.commit();
            sendBaseCodeInfo(BIDfromEhub);
            baseCodeName = newBaseCodeName;
          }
        }
        lastUpdateTime = millis();
      }

      if (BIDfromEhub == BID && mcuCodeWrite == 1 ) {
        String newMcuCodeName = docBattery["mcu_code"].as<String>();
        if(newMcuCodeName.length() < 31) // reserved 1 character for null terminator
        {
          if(newMcuCodeName != mcuCodeName)
          {
            EEPROM.writeString(EEPROM_MCU_CODE_ADDRESS, newMcuCodeName);
            EEPROM.write(EEPROM_MCU_ADDRESS_CONFIGURED_FLAG, 1);
            EEPROM.commit();
            sendMcuCodeInfo(BIDfromEhub);
            mcuCodeName = newMcuCodeName;
          }
        }
        lastUpdateTime = millis();
      }

      if (BIDfromEhub == BID && siteLocationWrite == 1 ) {
        String newSiteLocationname = docBattery["site_location"].as<String>();
        if(newSiteLocationname.length() < 31) // reserved 1 character for null terminator
        {
          if(newSiteLocationname != siteLocationName)
          {
            EEPROM.writeString(EEPROM_SITE_LOCATION_ADDRESS, newSiteLocationname);
            EEPROM.write(EEPROM_SITE_LOCATION_CONFIGURED_FLAG, 1);
            EEPROM.commit();
            sendSiteLocationInfo(BIDfromEhub);
            siteLocationName = newSiteLocationname;
          }
        }
        lastUpdateTime = millis();
      }

      if (BIDfromEhub == BID && rbq == 1)
      {
        getBQStatus(BIDfromEhub);
        lastUpdateTime = millis();
      }

      if (BIDfromEhub == BID && sbq == 1)
      {
        bqShutdown(BIDfromEhub);
        lastUpdateTime = millis();
      }

      if ((BIDfromEhub == BID || BIDfromEhub == 255) && restart == 1)
      {
        lastUpdateTime = millis();
        cmsRestart();
      }

      if (BIDfromEhub == BID && wbq == 1)
      {
        bqWakeUp(BIDfromEhub);
        lastUpdateTime = millis();
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
