#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "ICM_20948.h"
 
//BLE Globals
#define bleServerName "SwingSmart_ESP32"
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42f59"
volatile bool deviceConnected = false;
// https://www.uuidgenerator.net/
BLECharacteristic SSq0Characteristic("cba1d466-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor SSq0Descriptor(BLEUUID((uint16_t)0x2902));
BLECharacteristic SSq1Characteristic("6e581afe-8f8d-4ae2-8453-8d1e439a78f3", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor SSq1Descriptor(BLEUUID((uint16_t)0x2903));
BLECharacteristic SSq2Characteristic("ff708360-5cc4-49e9-a326-1090a5ae6622", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor SSq2Descriptor(BLEUUID((uint16_t)0x2904));
BLECharacteristic SSq3Characteristic("287171aa-6de7-4939-b4e8-e6d15b8f85a2", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor SSq3Descriptor(BLEUUID((uint16_t)0x2905));
//End BLE Globals
 
//IMU Globals
#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 5     // Which pin you connect CS to. Used only when "USE_SPI" is defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
 
#define GREEN_LED_PIN 10
#define RED_LED_PIN 9
//End IMU Globals
 
//Setup callbacks onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};
 
void setup() {
  // Start serial communication
  Serial.begin(115200);
 
  delay(100);
 
  Serial.println("Starting SPI port");
  SPI_PORT.begin();
 
  bool initialized = false;
  while (!initialized)
  {
    // Initialize the ICM-20948
    myICM.begin(CS_PIN, SPI_PORT);
 
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
 
  Serial.println("SPI port initialized");
  bool success = true;
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  if(!success)
  {
    Serial.println("DMP init failed");
  }
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
    if(!success)
  {
    Serial.println("DMP sensor enable failed");
  }
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok);
    if(!success)
  {
    Serial.println("DMP ODRate set failed");
  }
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok); // Enable the FIFO
    if(!success)
  {
    Serial.println("FIFO enable failed");
  }
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);  // Enable the DMP
    if(!success)
  {
    Serial.println("DMP enable failed");
  }
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);   // Reset DMP
    if(!success)
  {
    Serial.println("DMP reset failed");
  }
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);  // Reset FIFO
    if(!success)
  {
    Serial.println("FIFO reset failed");
  }
 
  // Check success
  if (success)
  {
    Serial.println("DMP enabled");
  }
  else
  {
    Serial.println(F("Enable DMP failed!"));
    Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1);
  }
 
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
 
  // Create the BLE Device
  BLEDevice::init(bleServerName);
  Serial.println("BLE device created");
 
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
 
  // Create the BLE Service
  BLEService *SwingService = pServer->createService(SERVICE_UUID);
 
  // Create BLE Characteristics and Create a BLE Descriptor
  SwingService->addCharacteristic(&SSq0Characteristic);
  SSq0Descriptor.setValue("IMU quaternion 0");
  SSq0Characteristic.addDescriptor(&SSq0Descriptor);
 
  SwingService->addCharacteristic(&SSq1Characteristic);
  SSq1Descriptor.setValue("IMU quaternion 1");
  SSq1Characteristic.addDescriptor(&SSq1Descriptor);
 
  SwingService->addCharacteristic(&SSq2Characteristic);
  SSq2Descriptor.setValue("IMU quaternion 2");
  SSq2Characteristic.addDescriptor(&SSq2Descriptor);
 
  SwingService->addCharacteristic(&SSq3Characteristic);
  SSq3Descriptor.setValue("IMU quaternion 3");
  SSq3Characteristic.addDescriptor(&SSq3Descriptor);
 
  // Start the service
  SwingService->start();
  Serial.println("BLE service started");
 
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}
 
void loop() {
 
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);
  delay(10);
 
  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for GRV data so we should receive Quat6
    {
      // Scale to +/- 1
      double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
 
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
 
      SSq0Characteristic.setValue(q0);
      SSq0Characteristic.notify();
      SSq1Characteristic.setValue(q1);
      SSq1Characteristic.notify();
      SSq2Characteristic.setValue(q2);
      SSq2Characteristic.notify();
      SSq3Characteristic.setValue(q3);
      SSq3Characteristic.notify();
    }
  }
 
  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }
}