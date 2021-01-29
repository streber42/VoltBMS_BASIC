/*
  Copyright (c) 2019 Simp ECO Engineering
  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the
  "Software"), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish,
  distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so, subject to
  the following conditions:
  The above copyright notice and this permission notice shall be included
  in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

  This is a stripped down version of the SIMPBMS Chevy Volt code

*/

#include <Arduino.h>
#include <EEPROM.h>
#include <FlexCAN_T4.h>  //https://github.com/collin80/FlexCAN_Library

#include "BMSModuleManager.h"
#include "Logger.h"
#include "SerialConsole.h"
#include "config.h"

#define RESTART_ADDR 0xE000ED0C
#define READ_RESTART() (*(volatile uint32_t*)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t*)RESTART_ADDR) = (val))
#define CPU_REBOOT WRITE_RESTART(0x5FA0004)

BMSModuleManager bms;
SerialConsole console;
EEPROMSettings settings;

/////Version Identifier/////////
int firmver = 666;

// Simple BMS V2 wiring//  EDITED
const int KEY = 17;     // input 1 - high active
const int CHRG_EN = 6;  // output 1 - high active
const int GAUGE = 20;   // GUAGE LEVEL OUTPUT
const int led = 13;

byte bmsstatus = 0;
// bms status values
#define Boot 0
#define Ready 1
#define Drive 2
#define Charge 3
#define Error 5
//
int ErrorReason = 0;
float gaugelevel;

// gauge filtering
const int numReadings = 10;
int readings[numReadings];  // the readings from the analog input
int readIndex = 0;          // the index of the current reading
int total = 0;              // the running total
int average = 0;            // the average

// variables for output control
uint16_t SOH = 100;  // SOH place holder

char msgString[128];  // Array to store serial string

// variables for current calulation
float ampsecond;
unsigned long lasttime;
unsigned long loopTimeMain, looptime, UnderTime, looptime1, cleartime,
    loopTimeBalance = 0;  // ms
int balanceStep = 0;

// Variables for SOC calc
int SOC = 100;  // State of Charge
int SOCset = 0;
int SOCtest = 0;

/// charger variables
int maxchargepercent = 100;

// variables
int incomingByte = 0;
int x = 0;
bool balancecells = false;
int cellspresent = 0;

CAN_message_t msgCache[200];

// Debugging modes//////////////////
int debug = 1;
int candebug = 0;     // view can frames
int debugdigits = 3;  // amount of digits behind decimal for voltage reading

void loadSettings() {
  Logger::console(0, "Resetting to factory defaults");
  settings.version = EEPROM_VERSION;
  settings.batteryID =
      0x01;  // in the future should be 0xFF to force it to ask for an address
  settings.OverVSetpoint = 4.25f;
  settings.UnderVSetpoint = 2.0f;
  settings.ChargeVsetpoint = 4.0f;
  settings.ChargeHys =
      .11f;  // voltage drop required for charger to kick back on
  settings.OverTSetpoint = 65.0f;
  settings.UnderTSetpoint = -10.0f;
  settings.IgnoreTemp = 0;  // 0 - use both sensors, 1 or 2 only use that sensor
  settings.IgnoreVolt = 0.5;  //
  settings.balanceVoltage = 3.9f;
  settings.balanceHyst = 0.005f;
  settings.logLevel = 2;
  settings.CAP = 100;     // battery size in Ah
  settings.Pstrings = 1;  // fstrings in parallel used to divide voltage of pack
  settings.Scells = 12;   // Cells in series
  settings.socvolt[0] =
      3100;                  // Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[1] = 10;  // Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[2] =
      4100;                  // Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[3] = 90;  // Voltage and SOC curve for voltage based SOC calc
  settings.voltsoc = 1;      // SOC purely voltage based
  settings.gaugelow = 125;   // empty fuel gauge pwm EDITED
  settings.gaugehigh = 255;  // full fuel gauge pwm
  settings.chargerspd = 100;  // ms per message
  settings.UnderDur = 5000;   // ms of allowed undervoltage before throwing open
                              // stopping discharge.
  settings.disp =
      1;  // 1 - display is used 0 - mirror serial data onto serial bus
}

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;
CAN_message_t msg;
// CAN_message_t inMsg;
// CAN_filter_t filter;

void sendBalanceCommands();  // send CAN commands to balance cells

void gaugeupdate() {
  analogWrite(GAUGE,
              map(average, 0, 100, settings.gaugelow, settings.gaugehigh));
}

void printbmsstat() {
  SERIALCONSOLE.println();
  SERIALCONSOLE.println();
  SERIALCONSOLE.println();
  SERIALCONSOLE.print("BMS Status : ");

  SERIALCONSOLE.print(bmsstatus);
  switch (bmsstatus) {
    case (Boot):
      SERIALCONSOLE.print(" Boot ");
      break;

    case (Ready):
      SERIALCONSOLE.print(" Ready ");
      break;

    case (Drive):
      SERIALCONSOLE.print(" Drive ");
      break;

    case (Charge):
      SERIALCONSOLE.print(" Charge ");
      break;

    case (Error):
      SERIALCONSOLE.print(" Error ");
      SERIALCONSOLE.print(ErrorReason);
      break;
  }

  SERIALCONSOLE.print("  ");

  if (digitalRead(KEY) == HIGH) {
    SERIALCONSOLE.print("| Key ON |");
  } else {
    SERIALCONSOLE.print("| Key OFF |");
  }

  if (balancecells) {
    SERIALCONSOLE.print("| Balancing Active");
  }

  SERIALCONSOLE.print("  ");
  SERIALCONSOLE.print(cellspresent);
  SERIALCONSOLE.println();
  SERIALCONSOLE.print("Charger Enable:");
  if (digitalRead(CHRG_EN) == HIGH) {
    SERIALCONSOLE.print(" ON ");
  } else {
    SERIALCONSOLE.print(" OFF ");
  }
  SERIALCONSOLE.println();
  // Can0.mailboxStatus();
  // Can1.mailboxStatus();
}

void updateSOC() {
  if (SOCset == 0) {
    if (millis() > 9000) {
      bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
    }
    if (millis() > 10000) {
      SOC = map(uint16_t(bms.getAvgCellVolt() * 1000), settings.socvolt[0],
                settings.socvolt[2], settings.socvolt[1], settings.socvolt[3]);

      ampsecond =
          (SOC * settings.CAP * settings.Pstrings * 10) / 0.27777777777778;
      SOCset = 1;
      if (debug != 0) {
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.println(
            "//////////////////////////////////////// SOC SET "
            "////////////////////////////////////////");
      }
    }
  }
  if (settings.voltsoc == 1) {
    SOC = map(uint16_t(bms.getAvgCellVolt() * 1000), settings.socvolt[0],
              settings.socvolt[2], settings.socvolt[1], settings.socvolt[3]);

    ampsecond =
        (SOC * settings.CAP * settings.Pstrings * 10) / 0.27777777777778;
  }
  SOC = ((ampsecond * 0.27777777777778) /
         (settings.CAP * settings.Pstrings * 1000)) *
        100;
  if (SOC >= 100) {
    ampsecond =
        (settings.CAP * settings.Pstrings * 1000) /
        0.27777777777778;  // reset to full, dependant on given capacity. Need
                           // to improve with auto correction for capcity.
    SOC = 100;
  }

  if (SOC < 0) {
    SOC = 0;  // reset SOC this way the can messages remain in range for other
              // devices. Ampseconds will keep counting.
  }

  SERIALCONSOLE.print("  ");
  SERIALCONSOLE.print("mA");
  SERIALCONSOLE.print("  ");
  SERIALCONSOLE.print(SOC);
  SERIALCONSOLE.print("% SOC ");
  SERIALCONSOLE.print(ampsecond * 0.27777777777778, 2);
  SERIALCONSOLE.println("mAh");
}

void SOCcharged(int percent) {
  SOC = percent * (maxchargepercent / 100);
  ampsecond =
      (percent / 100 * settings.CAP * settings.Pstrings * 1000) /
      0.27777777777778;  // reset to full, dependant on given capacity. Need to
                         // improve with auto correction for capcity.
}

void sendcommand();  // send BICM trigger message

void sendBalanceCommands()  // send CAN commands to balance cells
{
  sendcommand();
  bms.balanceCells();
}

void requestBICMdata() {
  sendcommand();

  for (int c = 0; c < 8; c++) {
    msg.buf[c] = 0;
  }
  msg.id = 0x300;
  msg.len = 8;
  Can0.write(msg);

  // for (int c = 0; c < 8; c++) {
  //   msg.buf[c] = 0;
  // }
  msg.id = 0x310;
  msg.len = 5;
  Can0.write(msg);
}

void make460(CAN_message_t& msg) {
  msg.id = 0x460;
  msg.len = 6;
  msg.buf[0] = 0x8C;
  msg.buf[1] = 0x92;
  msg.buf[2] = 0x4C;
  msg.buf[3] = 0x92;
  msg.buf[4] = 0xCC;
  msg.buf[5] = 0x9E;
  msg.seq = true;
}

void make461(CAN_message_t& eight_cell_msg) {
  eight_cell_msg.id = 0x461;
  eight_cell_msg.len = 8;
  eight_cell_msg.seq = true;
  eight_cell_msg.buf[0] = 0x8C;
  eight_cell_msg.buf[1] = 0x9A;
  eight_cell_msg.buf[2] = 0x4C;
  eight_cell_msg.buf[3] = 0x9A;
  eight_cell_msg.buf[4] = 0xCC;
  eight_cell_msg.buf[5] = 0xAA;
  eight_cell_msg.buf[6] = 0x0C;
  eight_cell_msg.buf[7] = 0xAD;
}

void maketemp(CAN_message_t& temp_msg) {
  temp_msg.id = 0x7E0;
  temp_msg.len = 8;
  uint8_t empty_buffer[8] = {0x00};
  memcpy(temp_msg.buf, empty_buffer, sizeof(empty_buffer));
  temp_msg.buf[0] = 0x4D;
  temp_msg.buf[1] = 0x5E;
  temp_msg.buf[7] = 0x6C;
}

void canread1(const CAN_message_t& inMsg) {
  if (!balancecells) {
    if (Can0.write(inMsg) != 1) {
      SERIALCONSOLE.println("can0 write error");
    }
  } else {
    if (inMsg.id == 0x276) {
      CAN_message_t six_cell_msg;
      CAN_message_t eight_cell_msg;
      CAN_message_t temp_msg;
      make460(six_cell_msg);
      make461(eight_cell_msg);
      maketemp(temp_msg);
      for (int i = 0x0; i < 0xF; i++) {
        if ((i == 7) || (i == B)) {
          continue;
        } else if (0x1 < i && i < 0xC) {  // handle 8-cell frames
          eight_cell_msg.id = 0x460 + i;
          Can1.write(eight_cell_msg);
          eight_cell_msg.id = 0x470 + i;
          Can1.write(eight_cell_msg);
        } else {
          six_cell_msg.id = 0x460 + i;
          Can1.write(six_cell_msg);
          six_cell_msg.id = 0x470 + i;
          Can1.write(six_cell_msg);
        }
        temp_msg.id = 0x7E0 + i;
        Can1.write(temp_msg);
        while (Can1.events() > 0) {
        }
      }
      // CAN_message_t six_cell_msg;
      // make460(six_cell_msg);
      // six_cell_msg.id = 0x470;
      // six_cell_msg.buf[1] = 0x9F;
      // six_cell_msg.buf[2] = 0x0C;
      // six_cell_msg.buf[3] = 0x9D;
      // six_cell_msg.buf[4] = 0x0C;
      // six_cell_msg.buf[5] = 0x9E;
      // Can1.write(six_cell_msg);
      // make460(six_cell_msg);
      // six_cell_msg.id = 0x460;
      // six_cell_msg.len = 6;
      // six_cell_msg.buf[0] = 0x8C;
      // six_cell_msg.buf[1] = 0x92;
      // six_cell_msg.buf[2] = 0x4C;
      // six_cell_msg.buf[3] = 0x92;
      // six_cell_msg.buf[4] = 0xCC;
      // six_cell_msg.buf[5] = 0x9E;
      // six_cell_msg.seq = true;
      // Can1.write(six_cell_msg);
      // CAN_message_t temp_msg;
      // temp_msg.id = 0x7E0;
      // temp_msg.len = 8;
      // uint8_t empty_buffer[8] = {0x00};
      // memcpy(temp_msg.buf, empty_buffer, sizeof(empty_buffer));
      // temp_msg.buf[0] = 0x4D;
      // temp_msg.buf[1] = 0x5E;
      // temp_msg.buf[7] = 0x6C;
      // Can1.write(temp_msg);
      // CAN_message_t eight_cell_msg;
      // eight_cell_msg.id = 0x461;
      // eight_cell_msg.len = 8;
      // eight_cell_msg.seq = true;
      // eight_cell_msg.buf[0] = 0x8C;
      // eight_cell_msg.buf[1] = 0x9A;
      // eight_cell_msg.buf[2] = 0x4C;
      // eight_cell_msg.buf[3] = 0x9A;
      // eight_cell_msg.buf[4] = 0xCC;
      // eight_cell_msg.buf[5] = 0xAA;
      // eight_cell_msg.buf[6] = 0x0C;
      // eight_cell_msg.buf[7] = 0xAD;
      // Can1.write(eight_cell_msg);
      // eight_cell_msg.id = 0x471;
      // eight_cell_msg.buf[1] = 0xAB;
      // eight_cell_msg.buf[2] = 0x0C;
      // eight_cell_msg.buf[3] = 0xAF;
      // eight_cell_msg.buf[4] = 0x0C;
      // eight_cell_msg.buf[5] = 0xAC;
      // eight_cell_msg.buf[6] = 0x0C;
      // eight_cell_msg.buf[7] = 0xAB;
      // Can1.write(eight_cell_msg);
      // temp_msg.id = 0x7E1;
      // temp_msg.buf[1] = 0x63;
      // temp_msg.buf[7] = 0x72;
      // Can1.write(temp_msg);
      // 19376490,00000462,false,0,8,8C,8E,4C,8E,CC,99,0C,9A
      // 19377016,00000310,false,1,5,00,00,00,00,00,00,00,00
      // 19379234,000007E2,false,0,8,4D,53,0D,5F,00,00,00,CF
      // 19380137,00000472,false,0,8,8C,98,0C,9A,0C,99,0C,99
      // 19382913,00000463,false,0,8,8C,8E,4C,8E,CC,99,0C,9A
      // 19383825,00000473,false,0,8,8C,98,0C,9A,0C,99,0C,9A
      // 19384785,000007E3,false,0,8,4D,74,00,00,00,00,00,85
      // 19386488,00000464,false,0,8,8C,AD,4C,AB,CC,B0,0C,AE
      // 19387400,00000474,false,0,8,8C,AE,0C,B1,0C,B2,0C,B3
      // 19388376,000007E4,false,0,8,4D,6D,00,00,00,00,00,7F
      // 19391727,00000465,false,0,8,8C,9F,4C,9F,CC,AE,0C,B0
      // 19392631,00000475,false,0,8,8C,AD,0C,AC,0C,AE,0C,AE
      // 19393584,000007E5,false,0,8,4D,42,0D,4E,00,00,00,B0
      // 19397087,00000466,false,0,8,8C,AD,4C,AC,CC,AE,0C,AE
      // 19397991,00000476,false,0,8,8C,B0,0C,B1,0C,B1,0C,B4
      // 19398951,000007E6,false,0,8,4D,5B,00,00,00,00,00,6F
      // 19421421,00000468,false,0,8,8C,B2,4C,B0,CC,B6,0C,B5
      // 19422333,00000478,false,0,8,8C,B4,0C,B6,0C,B5,0C,BB
      // 19423309,000007E8,false,0,8,4D,73,00,00,00,00,00,89
      // 19426245,00000469,false,0,8,8C,B0,4C,AE,CC,C4,0C,C7
      // 19427165,00000479,false,0,8,8C,C6,0C,C6,0C,C4,0C,C9
      // 19428109,000007E9,false,0,8,4D,2E,0D,36,00,00,00,88
      // 19431141,0000046A,false,0,8,8C,AF,4C,AE,CC,B5,0C,B4
      // 19432045,0000047A,false,0,8,8C,B7,0C,B7,0C,B4,0C,B6
      // 19433013,000007EA,false,0,8,4D,53,00,00,00,00,00,6B
      // 19439700,0000046C,false,0,6,8C,AE,4C,AC,CC,B5,00,00
      // 19440500,0000047C,false,0,6,8C,B6,0C,B2,0C,B9,00,00
      // 19441508,000007EC,false,0,8,4D,53,00,00,00,00,00,6D
      // 19444372,0000046D,false,0,6,8C,B2,4C,B0,CC,C1,00,00
      // 19445180,0000047D,false,0,6,8C,BE,0C,BE,0C,C5,00,00
      // 19446164,000007ED,false,0,8,4D,43,00,00,00,00,00,5E
      // 19449379,0000046E,false,0,6,8C,B0,4C,B1,CC,B9,00,00
      // 19450171,0000047E,false,0,6,8C,B3,0C,BA,0C,BA,00,00
      // 19451180,000007EE,false,0,8,4D,62,00,00,00,00,00,7E
    }
  }
}

void canread(const CAN_message_t& inMsg) {
  // if (Can0.read(inMsg)) {
  // Read data: len = data length, buf = data byte(s)
  // if (debug == 1) {
  //   SERIALCONSOLE.

  // if (!balancecells) {
    if (Can1.write(inMsg) != 1) {
      SERIALCONSOLE.println("can1 write error");
    }
  // }

  if (inMsg.id >= 0x460 &&
      inMsg.id <
          0x480)  // do volt magic if ids are ones identified to be modules
  {
    // DISABLE debugging otherwise message ids take over window
    // Serial.println(inMsg.id, HEX);
    bms.decodecan(
        inMsg);  // do volt magic if ids are ones identified to be modules
  }
  if (inMsg.id >= 0x7E0 &&
      inMsg.id <
          0x7F0)  // do volt magic if ids are ones identified to be modules
  {
    bms.decodecan(
        inMsg);  // do volt magic if ids are ones identified to be modules
  }
  if (debug == 1) {
    if (candebug == 1) {
      Serial.print(millis());
      if ((inMsg.id & 0x80000000) ==
          0x80000000)  // Determine if ID is standard (11 bits) or extended (29
                       // bits)
        sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:",
                (inMsg.id & 0x1FFFFFFF), inMsg.len);
      else
        sprintf(msgString, ",0x%.3lX,false,%1d", inMsg.id, inMsg.len);

      Serial.print(msgString);

      if ((inMsg.id & 0x40000000) ==
          0x40000000) {  // Determine if message is a remote request frame.
        sprintf(msgString, " REMOTE REQUEST FRAME");
        Serial.print(msgString);
      } else {
        for (byte i = 0; i < inMsg.len; i++) {
          sprintf(msgString, ", 0x%.2X", inMsg.buf[i]);
          Serial.print(msgString);
        }
      }
      Serial.println();
    }
  }
  // 	return true;
  // } else {
  // 	return false;
  // }
}

float getChargeVSetpoint() {
  return settings.ChargeVsetpoint * (maxchargepercent / 100);
}

void sendcommand()  // send BICM trigger message
{
  msg.id = 0x200;
  msg.len = 3;
  msg.buf[0] = 0x02;
  msg.buf[1] = 0x00;
  msg.buf[2] = 0x00;
  Can0.write(msg);
}

void resetwdog() {
  // noInterrupts(); //   No - reset WDT
  // WDOG_REFRESH = 0xA602;
  // WDOG_REFRESH = 0xB480;
  // interrupts();
}

void btUpdate() {
  if (settings.disp == 1) {
    Serial3.print("SOC=  ");
    Serial3.print(SOC);
    Serial3.println("");
    Serial3.println("");
    Serial3.print("Temp=  ");
    Serial3.print((bms.getAvgTemperature() * 1.8) + 32, 1);
    Serial3.println("");
    Serial3.println("");
    Serial3.print("Pack Volts=  ");
    Serial3.print(bms.getPackVoltage(), 2);
    Serial3.println("");
    Serial3.println("");
    Serial3.print("Low Cell=  ");
    Serial3.print(bms.getLowCellVolt(), 3);
    Serial3.println("");
    Serial3.println("");
    Serial3.print("High Cell=  ");
    Serial3.print(bms.getHighCellVolt(), 3);
    Serial3.println("");
    Serial3.println("");
    Serial3.print("Cell Delta=  ");
    Serial3.println((bms.getHighCellVolt() - bms.getLowCellVolt()), 3);
    Serial3.print("Charger=  ");
    if (digitalRead(CHRG_EN) == HIGH) {
      Serial3.print(" ON ");
    } else {
      Serial3.print(" OFF ");
    }
    Serial3.println("");
    Serial3.print("Balancing=  ");
    if (balancecells == 1) {
      Serial3.print(" ON ");
    } else {
      Serial3.print(" OFF ");
    }
    Serial3.println("");
    Serial3.println("--------------------");
    Serial3.println("--------------------");
  } else {
    Serial3.println(".");
    Serial3.println(".");
    Serial3.println(".");
    Serial3.print("BMS Status : ");

    Serial3.print(bmsstatus);
    switch (bmsstatus) {
      case (Boot):
        Serial3.print(" Boot ");
        break;

      case (Ready):
        Serial3.print(" Ready ");
        break;

      case (Drive):
        Serial3.print(" Drive ");
        break;

      case (Charge):
        Serial3.print(" Charge ");
        break;

      case (Error):
        Serial3.print(" Error ");
        break;
    }

    Serial3.print("  ");

    if (digitalRead(KEY) == HIGH) {
      Serial3.print("| Key ON |");
    }
    if (balancecells == 1) {
      Serial3.print("|Balancing Active");
    }
    Serial3.print("  ");
    Serial3.print(cellspresent);
    Serial3.println(".");
    Serial3.print("Out:");
    Serial3.print(digitalRead(CHRG_EN));

    Serial3.print(" In:");
    Serial3.print(digitalRead(KEY));
    bms.printPackDetails(debugdigits, 1);

    // Spacer for BT terminal
    Serial3.println(".");
    Serial3.println(".");
    Serial3.println(".");
    Serial3.println(".");
    Serial3.println(".");
  }
}

void socFilter() {
  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = SOC;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;
  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }
  // calculate the average:
  average = total / numReadings;
}

void loop() {
  loopTimeMain = millis();  // get current loop time
  // while (canread())
  // {
  // }
  while (Can0.events() > 0) {}
  while (Can1.events() > 0) {}

  // MAIN STATE MACHINE
  switch (bmsstatus) {
    case (Boot):
      digitalWrite(CHRG_EN, LOW);  // turn off charger
      bmsstatus = Ready;
      break;

    case (Ready):

      digitalWrite(CHRG_EN, LOW);  // turn off charger

      if (bms.getAvgCellVolt() > settings.balanceVoltage) {
        if ((bms.getHighCellVolt() - bms.getLowCellVolt() >
             (settings.balanceHyst * 2.0)))  // start balancing at hyst value
        {
          balancecells = true;
        } else if (bms.getHighCellVolt() - bms.getLowCellVolt() <=
                   settings.balanceHyst)  // stop balancing at half hyst
        {
          balancecells = false;
        }
      } else {
        balancecells = false;
      }

      if (bms.getHighCellVolt() <
          (settings.ChargeVsetpoint -
           settings.ChargeHys))  // detect AC present for charging and check not
                                 // balancing
      {
        bmsstatus = Charge;
      }

      if (digitalRead(KEY) == HIGH)  // detect Key ON
      {
        balancecells = false;  // stop balancing
        bmsstatus = Drive;
      }
      break;

    case (Drive):

      if (digitalRead(KEY) == LOW)  // Key OFF
      {
        bmsstatus = Ready;
      }
      break;

    case (Charge):
      balancecells = false;

      digitalWrite(CHRG_EN, HIGH);  // enable charger

      /*if (bms.getAvgCellVolt() > settings.balanceVoltage &&
      bms.getHighCellVolt() - bms.getLowCellVolt() > (settings.balanceHyst
      * 2.0))
      {
              //balancecells = true; EDITED
              //bmsstatus = Ready; EDITED
      }
      */

      // RESET Charge AH
      if (bms.getHighCellVolt() > getChargeVSetpoint() ||
          bms.getHighTemperature() > settings.OverTSetpoint) {
        if (bms.getAvgCellVolt() >
            (getChargeVSetpoint() - settings.balanceHyst)) {
          SOCcharged(100);
        } else {
          SOCcharged(95);
        }

        digitalWrite(CHRG_EN, LOW);  // turn off charger
        bmsstatus = Ready;
      }
      break;

    case (Error):
      digitalWrite(CHRG_EN, LOW);  // turn off charger

      if (bms.getLowCellVolt() > settings.UnderVSetpoint &&
          bms.getHighCellVolt() < settings.OverVSetpoint) {
        bmsstatus = Ready;
      }
      break;
  }

  // main loop 1000ms
  if (loopTimeMain - looptime >= 1000)  // process sequence 1sec
  {
    looptime = loopTimeMain;  // reset loop time
    bms.getAllVoltTemp();

    // UV  check
    if (bms.getLowCellVolt() < settings.UnderVSetpoint ||
        bms.getHighCellVolt() < settings.UnderVSetpoint) {
      if (UnderTime > millis())  // check is last time not undervoltage is
                                 // longer thatn UnderDur ago   murderdeathkill
      {
        bmsstatus = Error;
        ErrorReason = 2;
      }
    } else {
      UnderTime = millis() + settings.UnderDur;
    }

    SERIALCONSOLE.print("\033[1J");
    printbmsstat();
    bms.printPackDetails(debugdigits, 0);
    updateSOC();
    socFilter();
    gaugeupdate();

    // if (!balancecells)
    //   requestBICMdata();  // request data here only if not balancing.

    if (cellspresent == 0 && SOCset == 1) {
      cellspresent = bms.seriescells();
      bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
    } else {
      if (cellspresent != bms.seriescells() ||
          cellspresent !=
              (settings.Scells *
               settings.Pstrings))  // detect a fault in cells detected
      {
        // SERIALCONSOLE.println("  ");
        // SERIALCONSOLE.print("   !!! Series Cells Fault !!!");
        // SERIALCONSOLE.println("  ");
        // bmsstatus = Error;
        // ErrorReason = 3;
      }
    }
    // btUpdate();
    resetwdog();
  }

  // can loop 200ms
  if (loopTimeMain - loopTimeBalance >= 200) {
    loopTimeBalance = loopTimeMain;            // reset loop time
    if (balancecells && loopTimeMain > 15000)  // delay balancing
    {
      if (balanceStep < 10) {
        requestBICMdata();
        balanceStep++;
      } else if (balanceStep == 10) {
        bms.updateBalanceCells();
        sendBalanceCommands();
        balanceStep++;
      } else if (balanceStep > 10) {
        sendBalanceCommands();
        if (balanceStep >= 60) {
          balanceStep = 0;
          bms.clearBalanceCells();
        } else {
          balanceStep++;
        }
      }
    }
  }

  if (millis() - cleartime > 5000) {
    cleartime = millis();
    // bms.updateBalanceCells();
  }

  if (loopTimeMain - looptime1 > settings.chargerspd) {
    looptime1 = loopTimeMain;
  }
}

void setup() {
  delay(4000);  // just for easy debugging. It takes a few seconds for USB to
                // come up properly on most OS's
  pinMode(KEY, INPUT_PULLDOWN);
  pinMode(CHRG_EN, OUTPUT);  // charge relay
  pinMode(GAUGE, OUTPUT);    // GAUGE LEVEL OUTPUT
  pinMode(led, OUTPUT);

  analogWriteFrequency(GAUGE, 1000);

  Can0.begin();
  Can0.setBaudRate(125000);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(FIFO, canread);

  Can1.begin();
  Can1.setBaudRate(125000);
  Can1.enableFIFO();
  Can1.enableFIFOInterrupt();
  Can1.onReceive(FIFO, canread1);

  // set filters for standard
  // for (int i = 0; i < 8; i++)
  // {
  // 	Can0.getFilter(filter, i);
  // 	filter.flags.extended = 0;
  // 	Can0.setFilter(filter, i);
  // }
  // set filters for extended
  // for (int i = 9; i < 13; i++)
  // {
  // 	Can0.getFilter(filter, i);
  // 	filter.flags.extended = 1;
  // 	Can0.setFilter(filter, i);
  // }

  // filter setup
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  /////////////////

  SERIALCONSOLE.begin(115200);
  SERIALCONSOLE.println("Starting up!");
  SERIALCONSOLE.println("SimpBMS V2 Volt-Ampera");

  // Serial2.begin(115200);
  Serial3.begin(500000);

  // Display reason the Teensy was last reset
  // Serial.println();
  // Serial.println("Reason for last Reset: ");

  // if (RCM_SRS1 & RCM_SRS1_SACKERR)
  // 	Serial.println("Stop Mode Acknowledge Error Reset");
  // if (RCM_SRS1 & RCM_SRS1_MDM_AP)
  // 	Serial.println("MDM-AP Reset");
  // if (RCM_SRS1 & RCM_SRS1_SW)
  // 	Serial.println("Software Reset"); // reboot with SCB_AIRCR = 0x05FA0004
  // if (RCM_SRS1 & RCM_SRS1_LOCKUP)
  // 	Serial.println("Core Lockup Event Reset");
  // if (RCM_SRS0 & RCM_SRS0_POR)
  // 	Serial.println("Power-on Reset"); // removed / applied power
  // if (RCM_SRS0 & RCM_SRS0_PIN)
  // 	Serial.println("External Pin Reset"); // Reboot with software download
  // if (RCM_SRS0 & RCM_SRS0_WDOG)
  // 	Serial.println("Watchdog(COP) Reset"); // WDT timed out
  // if (RCM_SRS0 & RCM_SRS0_LOC)
  // 	Serial.println("Loss of External Clock Reset");
  // if (RCM_SRS0 & RCM_SRS0_LOL)
  // 	Serial.println("Loss of Lock in PLL Reset");
  // if (RCM_SRS0 & RCM_SRS0_LVD)
  // 	Serial.println("Low-voltage Detect Reset");
  // Serial.println();
  ///////////////////

  // enable WDT
  // noInterrupts();                 // don't allow interrupts while setting up
  // WDOG WDOG_UNLOCK = WDOG_UNLOCK_SEQ1; // unlock access to WDOG registers
  // WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  // delayMicroseconds(1); // Need to wait a bit..

  // WDOG_TOVALH = 0x1000;
  // WDOG_TOVALL = 0x0000;
  // WDOG_PRESC = 0;
  // WDOG_STCTRLH |= WDOG_STCTRLH_ALLOWUPDATE |
  // 	WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN |
  // 	WDOG_STCTRLH_STOPEN | WDOG_STCTRLH_CLKSRC;
  // interrupts();
  /////////////////

  SERIALCONSOLE.println("Started serial interface");

  EEPROM.get(0, settings);
  if (settings.version != EEPROM_VERSION) {
    loadSettings();
  }

  Logger::setLoglevel(
      Logger::Off);  // Debug = 0, Info = 1, Warn = 2, Error = 3, Off = 4

  digitalWrite(led, HIGH);
  bms.setPstrings(settings.Pstrings);
  bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
}
