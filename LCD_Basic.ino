#include "BLEDevice.h"
#include <Wire.h>

#define RSPin 12
#define WRPin 27
#define ENPin 14
#define DB0 9
#define DB1 10
#define DB2 5
#define DB3 18
#define DB4 23
#define DB5 19  
#define DB6 22
#define DB7 21

int i = 0;
int j = 1;
int k = 2;

void writeString(char sMessage[])
{
  sendCommand(0x01);
  for(int i = 0; i < 80; i++)
  {
    if(sMessage[i] == '.')
    {
      break;
    }
    writeScreen(sMessage[i]);
  }
}

void writePort(char portVal)
{
  digitalWrite(DB0, portVal & 0x01);
  digitalWrite(DB1, (portVal >> 1) & 0x01);
  digitalWrite(DB2, (portVal >> 2) & 0x01);
  digitalWrite(DB3, (portVal >> 3) & 0x01);
  digitalWrite(DB4, (portVal >> 4) & 0x01);
  digitalWrite(DB5, (portVal >> 5) & 0x01);
  digitalWrite(DB6, (portVal >> 6) & 0x01);
  digitalWrite(DB7, (portVal >> 7) & 0x01);
}

void sendCommand(char commandValue)
{
  Serial.println(commandValue, HEX);
  writePort(commandValue);
  digitalWrite(RSPin, 0);
  digitalWrite(WRPin, 0);
  digitalWrite(ENPin, 1);
  delay(2);
  digitalWrite(ENPin, 0);
}

void writeScreen(char writeValue)
{
  Serial.println(writeValue, HEX);
  //checkBF();
  writePort(writeValue);
  digitalWrite(RSPin, 1);
  digitalWrite(WRPin, 0);
  digitalWrite(ENPin, 1);
  delay(2);
  digitalWrite(ENPin, 0);
  delay(2);
}

void initLCD()
{
  // initialize LCD pins
  pinMode(RSPin, OUTPUT);
  pinMode(WRPin, OUTPUT);
  pinMode(ENPin, OUTPUT);
  pinMode(DB0, OUTPUT);
  pinMode(DB1, OUTPUT);
  pinMode(DB2, OUTPUT);
  pinMode(DB3, OUTPUT);
  pinMode(DB4, OUTPUT);
  pinMode(DB5, OUTPUT);
  pinMode(DB6, OUTPUT);
  pinMode(DB7, OUTPUT);

  // begin LCD initialization command codes
  digitalWrite(ENPin, 0);
  //Wait >40 msec after power is applied
  delay(100);
  //put 0x30 on the output port
  sendCommand(0x30);
  delay(30);
  sendCommand(0x30);
  delay(10);
  sendCommand(0x30);
  delay(10);

  sendCommand(0x3C); //Function set: 4-bit/2-line //  0x2c / 0x38
  sendCommand(0x0F); //Display ON; Blinking cursor // 0x0f / 0x0c
  sendCommand(0x04); //Entry Mode set //0x06
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  initLCD();
  delay(500);
  sendCommand(0x01);
}

void loop()
{
  char buffer[80];
  // put your main code here, to run repeatedly:
  sprintf(buffer, "PITCH: %d YAW: %d ROLL: %d.", i++, j++, k++);
  writeString(buffer);
  delay(80);
}
