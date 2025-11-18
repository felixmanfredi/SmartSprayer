#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "config.h"

BLEUUID HERMATECHSERVICE_UUID = BLEUUID((uint16_t)0x180A);
BLEUUID BATTERYSERVICE_UUID = BLEUUID((uint16_t)0x180F);

// Characteristics instantiation
BLECharacteristic SerialNumberCharacteristic((uint16_t)0x2A25, BLECharacteristic::PROPERTY_READ);
BLECharacteristic TelemetryCharacteristic(TELEMETRY_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic CommandsCharacteristic(COMMANDS_UUID, BLECharacteristic::PROPERTY_WRITE);
BLECharacteristic CommandResponseCharacteristic((uint16_t)0x2B2C, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic ErrorCodeCharacteristic((uint16_t)0x2BBB, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic BatteryCharacteristic((uint16_t)0x2A19, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);


// Descriptors instantiation
BLEDescriptor TelemetryDescriptor = BLEDescriptor((uint16_t)0x2912, 200); // UUID, max length of the descriptor

float LinearAcceleration[3] = {1.25, 13.65, 65.984};  // [m/s^2]
float AngularSpeed[3] = {1.98, 65.21, 0.4789};        // [rad/s]
float MagneticField[3] = {45.32, 321655.999, 10.20};  // [H]
float ObjectDistance = 10.51;                         // [cm]
float AmbientTemperature = 25.51;                     // [°C]
float AmbientHumidity = 50.51;                        // [%]
float AmbientPressure = 1.51;                         // [bar]

uint64_t ChipID = ESP.getEfuseMac(); //The chip ID is essentially its MAC address(length: 6 bytes).

///////////
uint8_t i = 0;
//////////////


class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    std::string value = pCharacteristic->getValue(); // Get the written data
     
    if (value.length() > 0) {
      Serial.print("Received command: ");
      Serial.println(value.c_str());
    }
  }
};



void setup() {

  // Initialize the serial connection
  Serial.begin(115200);
  // wait for Serial to come online (only for testing/development purposes)
  while (!Serial);
  
  delay(DELAY_AFTER_SERIAL_INITIALIZED);
  Serial.println("Serial start");

  // Create BLE Device
  BLEDevice::init(BLE_SERVER_NAME);

  // Request max MTU (server side)
  esp_err_t MTU_size = BLEDevice::setMTU(512);    // maximum value

  // Create BLE Server
  BLEServer *pHermaTechServer = BLEDevice::createServer();
  // pHermaTechServer->setCallbacks(new MyBLEServerCallbacks());


  // Create BLE Service

  //  Specificare un numero  di handles maggiore o uguale a quello effettivo. Considerare 1 handle per il Primary Service, 2 handle per ogni caratteristica (dichiarazione + valore),  1 handle per ogni descrittore extra di una caratteristica
  BLEService *pHermaTechService = pHermaTechServer->createService(HERMATECHSERVICE_UUID, 30, 0);

  BLEService *pBatteryService = pHermaTechServer->createService(BATTERYSERVICE_UUID, 5, 0);

  // Add characteristic and descriptors. Set the values
  pHermaTechService->addCharacteristic(&SerialNumberCharacteristic);
  SerialNumberCharacteristic.setValue(String(ChipID).c_str());

  pHermaTechService->addCharacteristic(&TelemetryCharacteristic);
  TelemetryDescriptor.setValue("Linear acceleration [X, Y, Z], angular speed [X, Y, Z], magnetic field strength [X, Y, Z], object distance, ambient temperature, ambient humidity, ambient pressure");
  TelemetryCharacteristic.addDescriptor(&TelemetryDescriptor);

  pHermaTechService->addCharacteristic(&CommandsCharacteristic);
  CommandsCharacteristic.setCallbacks(new CommandCallbacks());

  pHermaTechService->addCharacteristic(&CommandResponseCharacteristic);

  pHermaTechService->addCharacteristic(&ErrorCodeCharacteristic);
  

  pBatteryService->addCharacteristic(&BatteryCharacteristic);

  // Start the services
  pHermaTechService->start();
  pBatteryService->start();

  // Configure the advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(HERMATECHSERVICE_UUID);
  pAdvertising->addServiceUUID(BATTERYSERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  // BLEDevice::startAdvertising();
  pAdvertising->start();

  Serial.println("Set-up finished!");
}

void loop() {

  String SensorData;
  SensorData = "[" + String(LinearAcceleration[0] + i/10) + "," + String(LinearAcceleration[1]-i/10) + "," + String(LinearAcceleration[2]+i) + "] [m/s^2]";
  SensorData = SensorData + ", [" + String(AngularSpeed[0] + sqrt(i)) + "," + String(AngularSpeed[1] - sqrt(i)) + "," + String(AngularSpeed[2] / sqrt(i)) + "] [rad/s]";
  SensorData = SensorData + ", [" + String(MagneticField[0] * i) + "," + String(MagneticField[1] / i) + "," + String(MagneticField[2]* (i ^ 2)) + "] [H]";
  SensorData = SensorData + ", " + String(ObjectDistance * i) + " [cm]";
  SensorData = SensorData + ", " + String(AmbientTemperature + i) + " [°C]";
  SensorData = SensorData + ", " + String(AmbientHumidity - 0.3 * i) + " [%]";
  SensorData = SensorData + ", " + String(AmbientPressure + 0.5 * i) + " [bar]";

  Serial.println(SensorData);

  TelemetryCharacteristic.setValue(SensorData.c_str());
  TelemetryCharacteristic.notify();
  delay(200);

  CommandResponseCharacteristic.setValue(String(-i).c_str());
  CommandResponseCharacteristic.notify();
  delay(200);

  ErrorCodeCharacteristic.setValue(String(i).c_str());
  ErrorCodeCharacteristic.notify();
  delay(200);

  BatteryCharacteristic.setValue(String(i%100).c_str());
  BatteryCharacteristic.notify();
  delay(200);

  i++;
}



// #include <Arduino.h>
// #include <BLEDevice.h>
// #include <BLEServer.h>
// #include <BLEUtils.h>
// #include <BLE2902.h>
// #include "config.h"

// BLECharacteristic AccelometerCharacteristic(BLEUUID((uint16_t)0x2C06), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
// BLECharacteristic GyroscopeCharacteristic(BLEUUID((uint16_t)0x2C09), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
// BLECharacteristic MagnetometerCharacteristic(MAGNETOMETER_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
// BLECharacteristic ObjectDistanceCharacteristic(BLEUUID((uint16_t)0x2C15), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
// BLECharacteristic AmbientTemperatureCharacteristic(BLEUUID((uint16_t)0x2A6E), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
// BLECharacteristic AmbientHumidityCharacteristic(BLEUUID((uint16_t)0x2A6F), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
// BLECharacteristic AmbientPressureCharacteristic(BLEUUID((uint16_t)0x2A6D), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

// BLEDescriptor AccelometerDescriptor = BLEDescriptor(BLEUUID((uint16_t)0x2912));
// BLEDescriptor GyroscopeDescriptor = BLEDescriptor(BLEUUID((uint16_t)0x2912));
// BLEDescriptor MagnetometerDescriptor = BLEDescriptor(BLEUUID((uint16_t)0x2912));
// BLEDescriptor ObjectDistanceDescriptor = BLEDescriptor(BLEUUID((uint16_t)0x2912));

// uint64_t i =0;

// /* bool deviceConnected = false;

// // BLE Callbacks
// class MyBLEServerCallbacks : public BLEServerCallbacks
// {
//   void onConnect(BLEServer *pHermaTechServer)
//   {
//     deviceConnected = true;
//   };
//   void onDisconnect(BLEServer *pHermaTechServer)
//   {
//     deviceConnected = false;
//   }
// }; */

// void setup() 
// {
//   // Initialize the serial connection
//   Serial.begin(115200);
//   // wait for Serial to come online (only for testing/development purposes)
//   while (!Serial);
  
//   delay(DELAY_AFTER_SERIAL_INITIALIZED);
//   Serial.println("Serial start");

//   // Create BLE Device
//   BLEDevice::init(BLE_SERVER_NAME);

//   // Create BLE Server
//   BLEServer *pHermaTechServer = BLEDevice::createServer();
//   // pHermaTechServer->setCallbacks(new MyBLEServerCallbacks());

//   //  Specificare un numero  di handles maggiore o uguale a quello effettivo. 
//   //  Considerare 1 handle per il Primary Service, 2 handle per ogni caratteristica (dichiarazione + valore),  1 handle per ogni descrittore extra di una caratteristica
//   BLEService *pHermaTechService = pHermaTechServer->createService(BLEUUID((uint16_t)0x181A), 30, 0);

//   // Add characteristic and descriptors
//   pHermaTechService->addCharacteristic(&AccelometerCharacteristic);
//   AccelometerDescriptor.setValue("Linear acceleration [m/s] on X, Y, Z axes");
//   AccelometerCharacteristic.addDescriptor(&AccelometerDescriptor);

//   pHermaTechService->addCharacteristic(&GyroscopeCharacteristic);
//   GyroscopeDescriptor.setValue("Angular speed [rad/s] on X, Y, Z axes");
//   GyroscopeCharacteristic.addDescriptor(&GyroscopeDescriptor);

//   pHermaTechService->addCharacteristic(&MagnetometerCharacteristic);
//   MagnetometerDescriptor.setValue("Magnetic field strength [H] on X, Y, Z axes");
//   MagnetometerCharacteristic.addDescriptor(&MagnetometerDescriptor);
  
//   pHermaTechService->addCharacteristic(&ObjectDistanceCharacteristic);
//   ObjectDistanceDescriptor.setValue("Object distance from LIDAR [cm]");
//   ObjectDistanceCharacteristic.addDescriptor(&ObjectDistanceDescriptor);

//   pHermaTechService->addCharacteristic(&AmbientTemperatureCharacteristic);

//   pHermaTechService->addCharacteristic(&AmbientHumidityCharacteristic);

//   pHermaTechService->addCharacteristic(&AmbientPressureCharacteristic);

//   // Start the service
//   pHermaTechService->start();
//   // Configure the advertising
//   BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
//   pAdvertising->addServiceUUID(BLEUUID((uint16_t)0x181A));
//   pAdvertising->setScanResponse(true);
//   pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
//   pAdvertising->setMinPreferred(0x12);
//   BLEDevice::startAdvertising();
//   Serial.println("Characteristic defined! Now you can read it in your phone!");
// }

// void loop() {

//   String LinearAcceleration = String(i) + ", " + String(i+1) + ", " + String(i+2);
//   AccelometerCharacteristic.setValue(LinearAcceleration.c_str());
//   AccelometerCharacteristic.notify();
//   delay(500);

//   String AngularSpeed = String(i) + ", " + String(i+1) + ", " + String(i+2);
//   GyroscopeCharacteristic.setValue(AngularSpeed.c_str());
//   GyroscopeCharacteristic.notify();
//   delay(500);

//   String MagneticField = String(i) + ", " + String(i-1) + ", " + String(i-2);
//   MagnetometerCharacteristic.setValue(MagneticField.c_str());
//   MagnetometerCharacteristic.notify();
//   delay(500);

//   String ObejctDistance = String(i*10);
//   ObjectDistanceCharacteristic.setValue(ObejctDistance.c_str());
//   ObjectDistanceCharacteristic.notify();
//   delay(500);

//   int16_t AmbientTemperature = (int16_t)((i + 5) * 100); // resolution 0.01 °C
//   uint8_t Temperature[2];
//   Temperature[0] = AmbientTemperature & 0xFF;
//   Temperature[1] = (AmbientTemperature >> 8) & 0xFF;  
//   AmbientTemperatureCharacteristic.setValue(Temperature, 2);
//   AmbientTemperatureCharacteristic.notify();
//   delay(500);

//   int16_t AmbientHumidity = (int16_t)((i) * 100); // resolution 0.01 %
//   uint8_t Humidity[2];
//   Humidity[0] = AmbientHumidity & 0xFF;
//   Humidity[1] = (AmbientHumidity >> 8) & 0xFF;  
//   AmbientHumidityCharacteristic.setValue(Humidity, 2);
//   AmbientHumidityCharacteristic.notify();
//   delay(500);

//   int32_t AmbientPressure = 1013; // // resolution 0.1 Pa, i.e. 101.3 Pa
//   uint8_t Pressure[4];
//   Pressure[0] = AmbientPressure & 0xFF;
//   Pressure[1] = (AmbientPressure >> 8) & 0xFF;  
//   Pressure[2] = (AmbientPressure >> 16) & 0xFF;
//   Pressure[3] = (AmbientPressure >> 24) & 0xFF;  
//   AmbientPressureCharacteristic.setValue(Pressure, 4);
//   AmbientPressureCharacteristic.notify();
//   delay(500);

//   i++;
// }
