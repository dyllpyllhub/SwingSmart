#include "BLEDevice.h"
#include <Wire.h>
#include <driver/dac.h>
 
// Start IMU calibration defines
struct quat
{
  double s;
  double v[3];
};
double v_dot(double a[3], double b[3]) {
  return ((a[0]*b[0])+(a[1]*b[1])+(a[2]*b[2]));
}
struct quat q_mult(quat a, quat b) {
  quat out;
  out.s = a.s*b.s - v_dot(a.v,b.v);
  out.v[0] = (b.v[0] * a.s) + a.v[0] * b.s + ((a.v[1]*b.v[2])-(b.v[1]*a.v[2]));
  out.v[1] = (b.v[1] * a.s) + a.v[1] * b.s + ((a.v[2]*b.v[0])-(b.v[2]*a.v[0]));
  out.v[2] = (b.v[2] * a.s) + a.v[2] * b.s + ((a.v[0]*b.v[1])-(b.v[0]*a.v[1]));
  return out;
}
struct quat q_invert(quat q) {
  quat out;
  out.s = q.s;
  out.v[0] = -q.v[0];
  out.v[1] = -q.v[1];
  out.v[2] = -q.v[2];
  return out;
}
struct quat qpq(quat q, quat p) {
  return q_mult(q,q_mult(p,q_invert(q)));
}
quat strike, out, k, i;
const double STRIKEFLAG = 2;
bool dataValid = false;
bool strikeDetected = false;
// End IMU defines
 
// Defines for audio output
hw_timer_t *Timer0_Cfg = NULL;
uint8_t SampleIdx = 0;
const int sineLookupTable[] = { 43,45,48,50,53,56,58,61,
                                63,65,67,70,72,73,75,77,
                                78,80,81,82,83,84,84,85,
                                85,85,85,85,84,84,83,82,
                                81,80,78,77,75,73,72,70,
                                67,65,63,61,58,56,53,50,
                                48,45,43,40,37,35,32,29,
                                27,24,22,20,18,15,13,12,
                                10,8,7,5,4,3,2,1,
                                1,0,0,0,0,0,1,1,
                                2,3,4,5,7,8,10,12,
                                13,15,18,20,22,24,27,29,
                                32,35,37,40,43 };
 
// End audio output defines
 
// Defines for LCD
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
 
char buffer[80];
// End LCD defines
 
// BLE define section
#define bleServerName "SwingSmart_ESP32"
// UUID's of the service and characteristic that we want to read
static BLEUUID SSIMUServiceUUID("91bad492-b950-4226-aa2b-4ede9fa42f59");
static BLEUUID SSq0UUID("cba1d466-344c-4be3-ab3f-189f80dd7518");
static BLEUUID SSq1UUID("6e581afe-8f8d-4ae2-8453-8d1e439a78f3");
static BLEUUID SSq2UUID("ff708360-5cc4-49e9-a326-1090a5ae6622");
static BLEUUID SSq3UUID("287171aa-6de7-4939-b4e8-e6d15b8f85a2");
// Flags stating if should begin connecting and if the connection is up
static boolean doConnect = false;
static boolean connected = false;
// Variable to hold the address of the secondary device
static BLEAddress *pServerAddress;
// Characteristicd that we want to read
static BLERemoteCharacteristic* SSq0Characteristic;
static BLERemoteCharacteristic* SSq1Characteristic;
static BLERemoteCharacteristic* SSq2Characteristic;
static BLERemoteCharacteristic* SSq3Characteristic;
// Decide whether we want to be notified. Will likely always be on
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t notificationOff[] = {0x0, 0x0};
// Variables to store received data
char* cq0;
double* dq0;
char* cq1;
double* dq1;
char* cq2;
double* dq2;
char* cq3;
double* dq3;
// Flags to check whether new readings are available
boolean newq0 = false;
boolean newq1 = false;
boolean newq2 = false;
boolean newq3 = false;
// End BLE defines section
 
/********************************************************************/
/********************* Start helper functions ***********************/
/********************************************************************/
 
// The Timer0 ISR
void IRAM_ATTR Timer0_ISR()
{
  // Send SineTable Values To DAC One By One
  dac_output_voltage(DAC_CHANNEL_1, sineLookupTable[SampleIdx++]);
  if(SampleIdx == 100)
  {
    SampleIdx = 0;
  }
}
 
void audioSystemInit()
{
  Timer0_Cfg = timerBegin(0, 80, true);                 // 80 is the prescaler value. Adjust it to achieve different
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);  // pitches / make a tune
  timerAlarmWrite(Timer0_Cfg, 10, true);
  timerAlarmEnable(Timer0_Cfg);
  dac_output_enable(DAC_CHANNEL_1);
}
 
void playNote(int pitch, int duration)
{
  // This function blocks other processes. Only use it to make sounds
  // when the sound is the main priority
 
  int adjustedPitch = pitch;  // later on, add the math to convert the pitch to a
                              // clock divider value
 
  timerSetDivider(Timer0_Cfg, adjustedPitch);
  timerAlarmEnable(Timer0_Cfg);
  delay(duration);
  timerAlarmDisable(Timer0_Cfg);
}
 
// Connect to the BLE Server that has the name, Service, and Characteristics
bool connectToServer(BLEAddress pAddress) {
   BLEClient* pClient = BLEDevice::createClient();
 
  // Connect to the remove BLE Server.
  pClient->connect(pAddress);
  Serial.println(" - Connected to server");
 
  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(SSIMUServiceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(SSIMUServiceUUID.toString().c_str());
    return (false);
  }
 
  // Obtain a reference to the characteristics in the service of the remote BLE server.
  SSq0Characteristic = pRemoteService->getCharacteristic(SSq0UUID);
  SSq1Characteristic = pRemoteService->getCharacteristic(SSq1UUID);
  SSq2Characteristic = pRemoteService->getCharacteristic(SSq2UUID);
  SSq3Characteristic = pRemoteService->getCharacteristic(SSq3UUID);
 
  if (SSq0Characteristic == nullptr || SSq1Characteristic == nullptr || SSq2Characteristic == nullptr || SSq3Characteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID");
    return false;
  }
  Serial.println(" - Found our characteristics");
 
  //Assign callback functions for the Characteristics
  SSq0Characteristic->registerForNotify(SSq0NotifyCallback);
  SSq1Characteristic->registerForNotify(SSq1NotifyCallback);
  SSq2Characteristic->registerForNotify(SSq2NotifyCallback);
  SSq3Characteristic->registerForNotify(SSq3NotifyCallback);
  return true;
}
 
//Callback function that gets called, when another device's advertisement has been received
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName() == bleServerName) { //Check if the name of the advertiser matches
      advertisedDevice.getScan()->stop(); //Scan can be stopped, we found what we are looking for
      pServerAddress = new BLEAddress(advertisedDevice.getAddress()); //Address of advertiser is the one we need
      doConnect = true; //Set indicator, stating that we are ready to connect
      Serial.println("Device found. Connecting!");
    }
  }
};
 
//When the BLE Server sends a new temperature reading with the notify property
static void SSq0NotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                                        uint8_t* pData, size_t length, bool isNotify) {
  dq0 = (double*)pData;
  newq0 = true;
}
 
static void SSq1NotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                                        uint8_t* pData, size_t length, bool isNotify) {
  dq1 = (double*)pData;
  newq1 = true;
}
 
static void SSq2NotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                                        uint8_t* pData, size_t length, bool isNotify) {
  dq2 = (double*)pData;
  newq2 = true;
}
 
static void SSq3NotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                                        uint8_t* pData, size_t length, bool isNotify) {
  dq3 = (double*)pData;
  newq3 = true;
}
 
// LCD Functions to display messages
void writeString(char sMessage[])
{
  sendCommand(0x01);
  for(int i = 0; i < 80; i++)
  {
    if(sMessage[i] == '`')
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
 
/********************************************************************/
/********************** End helper functions ************************/
/********************************************************************/
 
void setup() {
  //Start serial communication
  Serial.begin(115200);
 
  k.s = 0;
  k.v[0] = 0;
  k.v[1] = 0;
  k.v[2] = 1;
  i.s = 0;
  i.v[0] = 1;
  i.v[1] = 0;
  i.v[2] = 0;
 
  initLCD();
  delay(500);
  sendCommand(0x01);
 
  sprintf(buffer, "Initializing audio system`");
  writeString(buffer);
 
  audioSystemInit();
  playNote(80, 200);
  playNote(100, 500);
  dac_output_disable(DAC_CHANNEL_1);
 
  sprintf(buffer, "Starting BLE client tap app`");
  writeString(buffer);
 
  Serial.println("Starting Arduino BLE Client application...");
 
  //Init BLE device
  BLEDevice::init("");
 
  // Start scanning for a server
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  Serial.println("Beginning scan");
  pBLEScan->start(30);
}
 
void loop() {
  // Once connected, output this info to the serial connection, and to the LCD.
  // Then turn on the notification property
  if (doConnect == true) {
    if (connectToServer(*pServerAddress)) {
      Serial.println("We are now connected to the BLE Server.");
      sprintf(buffer, "Connected to BLE Server!`");
      writeString(buffer);
      SSq0Characteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
      SSq1Characteristic->getDescriptor(BLEUUID((uint16_t)0x2903))->writeValue((uint8_t*)notificationOn, 2, true);
      SSq2Characteristic->getDescriptor(BLEUUID((uint16_t)0x2904))->writeValue((uint8_t*)notificationOn, 2, true);
      SSq3Characteristic->getDescriptor(BLEUUID((uint16_t)0x2905))->writeValue((uint8_t*)notificationOn, 2, true);
      connected = true;
    } else {
      Serial.println("We have failed to connect to the server; Restart your device to scan for nearby BLE server again.");
    }
    doConnect = false;
  }
 
  if (newq0 && newq1 && newq2 && newq3)
  {
    dataValid = true;

    if(*dq0 == STRIKEFLAG && *dq1 == STRIKEFLAG && *dq2 == STRIKEFLAG && *dq3 == STRIKEFLAG)
    {
      dataValid = false;
      strikeDetected = true;
    }

    if(strikeDetected && dataValid)
    {
      strike.s = *dq0;
      strike.v[0] = *dq1;
      strike.v[1] = *dq2;
      strike.v[2] = *dq3;

      out = qpq(strike,k);
      double face = -asin(out.v[1])*(180/PI);
      double shaft = -asin(out.v[2])*(180/PI);

      out = qpq(strike,i);
      double stance = asin(out.v[1])*(180/PI);
      
      sprintf(buffer, "Swing Smart         Shaft Angle %+7.2lf Face Angle %+8.2lf Lean %+14.2lf`", shaft, face, stance);
      writeString(buffer);

      strikeDetected = false;
    }

    if(dataValid)
    {
      Serial.print(F("{\"quat_w\":"));
      Serial.print(*dq0, 3);
      Serial.print(F(", \"quat_x\":"));
      Serial.print(*dq1, 3);
      Serial.print(F(", \"quat_y\":"));
      Serial.print(*dq2, 3);
      Serial.print(F(", \"quat_z\":"));
      Serial.print(*dq3, 3);
      Serial.println(F("}"));
    }

    newq0 = false;
    newq1 = false;
    newq2 = false;
    newq3 = false;
  }
}