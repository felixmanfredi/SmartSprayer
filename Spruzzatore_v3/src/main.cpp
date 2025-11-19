#include <Arduino.h>
#include "ble_callbacks.h"
#include "utils.h"
#include <Wire.h>
#include <TFLI2C.h> 
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

const char *BLE_SERVER_NAME = "HermaTech";
const unsigned long SERIAL_BAUD = 115200;

// Sensore BME280 - Temperatura, Umidità, Pressione
Adafruit_BME280 bme;

// Caratteristica GATT che pubblica i dati dei sensori
const char *TELEMETRY_UUID = "38938a29-7798-4dd6-bca2-c8fc70f65bfc";

// Caratteristica GATT che riceve i comandi di controllo
const char *COMMANDS_UUID = "f7a40c1c-e012-4c18-b838-51de426a868c";

BLEUUID HERMATECHSERVICE_UUID = BLEUUID((uint16_t)0x180A);
BLEUUID BATTERYSERVICE_UUID = BLEUUID((uint16_t)0x180F);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, -1); 
TFLI2C lidar; 

// Variabili per i dati dei sensori
int16_t distance = 0; // Distanza letta dal lidar in cm
float temperature = 0.0; // Temperatura ambiente in °C
float humidity = 0.0; // Umidità relativa in %
float pressure = 0.0; // Pressione atmosferica in hPa

float vbat = 0.0; // Tensione batteria in V

// Istanze delle characteristic BLE
BLECharacteristic SerialNumberCharacteristic((uint16_t)0x2A25, BLECharacteristic::PROPERTY_READ); // Characteristic per il numero di serie
BLECharacteristic TelemetryCharacteristic(TELEMETRY_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY); // Characteristic per i dati dei sensori
BLECharacteristic CommandsCharacteristic(COMMANDS_UUID, BLECharacteristic::PROPERTY_WRITE); // Characteristic per i comandi di controllo
BLECharacteristic CommandResponseCharacteristic((uint16_t)0x2B2C, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY); // Characteristic per le risposte ai comandi
BLECharacteristic ErrorCodeCharacteristic((uint16_t)0x2BBB, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY); // Characteristic per i codici di errore

// Descrittore per la characteristic Telemetry (max 200 byte)
BLEDescriptor TelemetryDescriptor = BLEDescriptor((uint16_t)0x2912, 200); 



uint64_t ChipID = ESP.getEfuseMac(); 


// --- Stato Sistema ---
enum Page { PAGE_INFO, PAGE_LASER};
Page currentPage = PAGE_INFO;

bool bleConnected = false; 
bool lasersOn = false; 

// --- Encoder Rotativo ---
int lastClkState;
int currentClkState;

// --- Stato acceso/spento sistema ---
bool systemActive = false;

// --- Setup e Loop ---

void setup() {
  Serial.begin(115200);
  Wire1.begin(SCA_OLED, SCLA_OLED, 400000); 
  Wire.begin(SDA_I2C, SCL_I2C); 
  
  Serial.println("Inizializzazione...");
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); 
  }

  if(!bme.begin(0x77)) {  
    Serial.println("Impossibile trovare il sensore BME280. Controllare i collegamenti!");
    while (1);
  }

  analogReadResolution(12); 
  analogSetPinAttenuation(BATT_SENSE, ADC_11db);  

  // Inizializzo i pin
  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(EXT_LED, OUTPUT);

  // Lidar
  pinMode(CONF_LIDAR, OUTPUT);
  digitalWrite(CONF_LIDAR, LOW); 

  // Laser 
  pinMode(LASER_1, OUTPUT);
  pinMode(LASER_2, OUTPUT);

  // Pulsante ed encoder
  pinMode(PB_SWITCH, INPUT_PULLUP);

  pinMode(PB_ENCODER, INPUT_PULLUP);
  pinMode(DT_ENCODER, INPUT);
  pinMode(CLK_ENCODER, INPUT);
  lastClkState = digitalRead(CLK_ENCODER);

  // Pin per rilevare se i 5V esterni sono presenti
  pinMode(EXT_5V_SENSE, INPUT); 

  // Inizializzo BLE
  BLEDevice::init(BLE_SERVER_NAME);

  esp_err_t MTU_size = BLEDevice::setMTU(512); 
  Serial.print("MTU size set to: ");
  Serial.println(MTU_size);


  BLEServer *pHermaTechServer = BLEDevice::createServer();
  pHermaTechServer->setCallbacks(new MyServerCallbacks()); // Gestione connessioni/disconnessioni

  // --- HermaTech Service (custom) ---
  BLEService *pHermaTechService = pHermaTechServer->createService(HERMATECHSERVICE_UUID, 30, 0);

  pHermaTechService->addCharacteristic(&SerialNumberCharacteristic); // Characteristic per il numero di serie
  SerialNumberCharacteristic.setValue(String(ChipID).c_str()); // Imposto il valore del numero di serie 

  pHermaTechService->addCharacteristic(&TelemetryCharacteristic); // Characteristic per i dati dei sensori

  pHermaTechService->addCharacteristic(&CommandsCharacteristic); // Characteristic per i comandi di controllo
  CommandsCharacteristic.setCallbacks(new CommandCallbacks()); // Imposto i callback per la gestione dei comandi ricevuti via BLE

  pHermaTechService->addCharacteristic(&CommandResponseCharacteristic); // Characteristic per le risposte ai comandi
  
  pHermaTechService->addCharacteristic(&ErrorCodeCharacteristic); // Characteristic per i codici di errore

  // Avvio i servizi
  pHermaTechService->start();

  // Avvio della pubblicazione dei servizi
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising(); 
  pAdvertising->addServiceUUID(HERMATECHSERVICE_UUID); 
  pAdvertising->setScanResponse(true); 
  pAdvertising->setMinPreferred(0x06);  
  pAdvertising->setMinPreferred(0x12);
  pAdvertising->start(); 
  Serial.println("Servizi BLE pubblicati. In attesa di connessioni...");
  

  display.setTextColor(WHITE);
  display.clearDisplay(); 
  display.display(); 

}

void loop() {
  unsigned long now = millis();

  // Aggiorno lo stato del sistema
  updateSystemState(now);

  // Aggiorno i dati dei sensori
  lidar.getData(distance, 0x10); // 0x10 = cm
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F; 

  vbat = readBatteryVoltage();

  // Gestione encoder per cambiare pagina
  currentClkState = digitalRead(CLK_ENCODER);
  if (currentClkState != lastClkState) {
    if (digitalRead(DT_ENCODER) != currentClkState) {
      // rotazione oraria
      currentPage = PAGE_LASER;
    } else {
      // rotazione antioraria
      currentPage = PAGE_INFO;
    }
  }
  lastClkState = currentClkState;

  // Gestione pulsante PB_SWITCH per i laser
  static bool lastSwitchState = HIGH;
  bool switchState = digitalRead(PB_SWITCH);
  if (lastSwitchState == HIGH && switchState == LOW && currentPage == PAGE_LASER) {
    lasersOn = !lasersOn;
    digitalWrite(LASER_1, lasersOn ? HIGH : LOW);
    digitalWrite(LASER_2, lasersOn ? HIGH : LOW);
  }
  lastSwitchState = switchState;


  String SensorData;
  SensorData = "[" + String(temperature) + "[°C]," + String(humidity) + "[%]," + String(pressure) + "[hPa]"; // Temperatura [°C], Umidità [%], Pressione [hPa]
  SensorData = SensorData + "," + String(distance) + "[cm]"; // Distanza lidar [cm]
  SensorData = SensorData + ",BATT:" + String(batteryPercentLipo(vbat)) + "[%]"; // Tensione batteria [V]
  SensorData = SensorData + ",SYS:" + String(systemActive ? "ON" : "OFF");
  SensorData = SensorData + ",LASER:" + String(lasersOn ? "ON" : "OFF") + "]";


  Serial.println("SensorData: " + SensorData);
  if (bleConnected) {
    TelemetryCharacteristic.setValue(SensorData.c_str());
    TelemetryCharacteristic.notify();
  }

  // Aggiornamento display (ogni 200 ms)
  static unsigned long lastDisplayUpdate = 0;
  if (now - lastDisplayUpdate >= 200) {
    lastDisplayUpdate = now;
    if (currentPage == PAGE_INFO)
      drawInfoPage();
    else
      drawLaserPage();
  }

}