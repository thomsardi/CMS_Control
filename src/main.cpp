#include <Arduino.h>
#include <bq769x0.h>
#include <Wire.h>
#include <registers.h>
#include <function.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <FastLED.h>

#define BQ769x0 1

#define RXD2 16
#define TXD2 17
#define LED_PIN     19
#define NUM_LEDS    8

#define BMS_ALERT_PIN 34     // attached to interrupt INT0
#define BMS_BOOT_PIN 21      // connected to TS1 input
#define BMS_I2C_ADDRESS 0x08

#define EEPROM_FRAME_NAME_ADDRESS 0x00
#define EEPROM_CONFIGURED_FLAG 0x20

DynamicJsonDocument docBattery(768);
CRGB leds[NUM_LEDS];
bq769x0 BMS[3] = {bq769x0(bq76940, BMS_I2C_ADDRESS, 0), bq769x0(bq76940, BMS_I2C_ADDRESS, 1), bq769x0(bq76940, BMS_I2C_ADDRESS, 2)};
bool isFirstRun = true;
unsigned long currTime;
TwoWire wire = TwoWire(0);

const String firmwareVersion = "v0.1";
const String productionCode = "CMS-32-01";
const String chipType = "BQ769x0";

int sda = 26;
int scl = 27;

int BIDfromEhub;
int BID0 = 999;
int BID = 0 ;
const int addr = 23;
const int tombol = 18;
int buttonState = 0;
//int ReadBal = 0;
//int SetBal = 0;
//int ClearBal = 0;

int R;
int g;
int b;
//int rjson;
//int gjson;
//int bjson;
//int numled;
//int setled;
//int vpack;
//int temp;
//int rbq;
//int wbq;
//int sbq;
//int OffBal;
//int vcell;

String CMSFrameName = "undefined";

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

void ReadBalancing ()
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
  pinMode(addr, INPUT );
  pinMode(tombol, INPUT );
  for (int i = 0; i < 3; i ++)
  {
    BMS[i].setI2C(&wire);
  }

  Scanner();
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

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Multiple BMS Example");
  DynamicJsonDocument docBattery(768);
  deserializeJson(docBattery, Serial2);
  JsonObject object = docBattery.as<JsonObject>();
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
menu:
  while (1) {
    DynamicJsonDocument docBattery(768);
    deserializeJson(docBattery, Serial2);
    JsonObject object = docBattery.as<JsonObject>();

    if (!object.isNull()) {
      BIDfromEhub = docBattery["BID"];
      if (BIDfromEhub == BID0  ) {
        String output;
        docBattery["BID"] = BID0;
        // serializeJson(docBattery, Serial2);
        serializeJson(docBattery, output);
        Serial2.print(output);
        Serial2.print('\n');
        Serial.println("NOBID");
        goto menu3;
      }

      if (BIDfromEhub == 619)
      {
        // Serial2.println("Secret Code");
        goto menu2;
      }
    }

    if (digitalRead(addr) == HIGH) {
      DynamicJsonDocument docBattery(768);
      String output;
      docBattery["BID"] = "?";
      // serializeJson(docBattery, Serial2);
      serializeJson(docBattery, output);
      Serial2.print(output);
      Serial2.print('\n');
      // Serial2.println("GETID");
      delay(200);
      goto menu1;
    }
  }

menu1:
  while (1) {
    DynamicJsonDocument docBattery(768);
    deserializeJson(docBattery, Serial2);
    JsonObject object = docBattery.as<JsonObject>();
    BID = docBattery["BID"];
    if (BID > 0) {
      BID = docBattery["BID"];
      DynamicJsonDocument docBattery(768);
      String output;
      docBattery["BID"] = BID;
      docBattery["RESPON"] = BID;
      // serializeJson(docBattery, Serial2);
      serializeJson(docBattery, output);
      Serial2.print(output);
      Serial2.print('\n');
      //      Serial.println(BID);
      int no = BID - 1;
      leds[no] = CRGB::LightSeaGreen;
      FastLED.setBrightness(20);
      FastLED.show();
      delay(200);
      goto menu2;
    }
  }

menu2:
  while (1) {
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
    DynamicJsonDocument docBattery(1024);
    deserializeJson(docBattery, Serial2);
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
    }

    if (BIDfromEhub == BID && vpack == 1)
    {
      GetVpack();
      //      GetDeviceStatus();
    }

    if (BIDfromEhub == BID && temp == 1)
    {
      GetTemp();
    }
    
    if ( BIDfromEhub == BID && ReadBal == 1 ) {
      ReadBalancing();
    }

    if (BIDfromEhub == BID && SetBal > 0 ) {
      SetBalancing(SetBal);
    }

    if (BIDfromEhub == BID && OffBal > 0)
    {
      OffBalancing(OffBal);
    }

    if (BIDfromEhub == BID && ClearBal == 1 ) {
      ClearBalancing();
    }

    if ( BIDfromEhub == BID && setled == 1 )
    {
      leds[numled] = CRGB(rjson, gjson, bjson);
      FastLED.show();
      DynamicJsonDocument doc(64);
      doc["BID"] = BID;
      doc["LEDSET"] = 1;
      doc["L"] = BID;
      doc["STATUS"] = 1;
      String output;
      serializeJson(doc, output);
      Serial2.print(output);
      Serial2.print('\n');
//      serializeJson(docBattery, Serial2);
//      Serial2.println(output);
    }

    if (BIDfromEhub == BID && info == 1 ) {
      // String output;
      sendInfo(BIDfromEhub);
      // Serial2.print(output);
      // Serial2.print('\n');
    } 

    if (BIDfromEhub == BID && frameWrite == 1 ) {
      // String output;
      // Serial2.println("Frame Write Processing");
      CMSFrameName = docBattery["frame_name"].as<String>();
      EEPROM.writeString(EEPROM_FRAME_NAME_ADDRESS, CMSFrameName);
      EEPROM.write(EEPROM_CONFIGURED_FLAG, 1);
      sendFrameInfo(BIDfromEhub);
      EEPROM.commit();
      // Serial2.print(output);
      // Serial2.print('\n');
    } 

    if (BIDfromEhub == BID && rbq == 1)
    {
      GetDeviceStatus();
    }

    if (BIDfromEhub == BID && sbq > 0)
    {
      bqShut(sbq);
    }

    if (BIDfromEhub == BID && wbq > 0 )
    {
      bqWake(wbq);
    }
  }

menu3:
  while (1) {
    Serial.println("minta No id ulang");
    DynamicJsonDocument docBattery(768);

    docBattery["BID"] = "noBID";
    String output;
    // serializeJson(docBattery, Serial2);
    serializeJson(docBattery, output);
    Serial2.print(output);
    Serial2.print('\n');
    delay(100);
    goto menu;
  }
}
