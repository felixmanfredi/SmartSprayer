#include <config.h>  // Includes configuration file containing constants, pin definitions, and settings.

//------------------------------//
//       GLOBAL VARIABLES       //
//------------------------------//

volatile unsigned int pulseCount; 
// Stores the number of wind speed sensor pulses (modified inside an ISR, hence volatile).

unsigned int count, i = 0, j = 0;
// 'count' for temporary pulse storage; 'i' and 'j' as array indices for averaging.

float BatteryLevel, VoltageSensing, WindSpeed[SPEED_SAMPLE_NUMBER] = {0};
// Battery voltage, sensed voltage value, and array to store recent wind speed samples.

unsigned int WindDirection[DIRECTION_SAMPLE_NUMBER] = {0};
// Array storing recent analog readings for wind direction averaging.

unsigned long currentTime, lastSampleTime, firstInterruptTime, lastInterruptTime, StartTime_WiFiTimeout;
// Timestamps for pulse timing, sampling intervals, and WiFi timeout logic.

bool FirstTime_Speed = HIGH, FirstTime_Direction = HIGH, NotBack = HIGH;
// Flags for managing first iteration and WiFi state tracking.

String TelemetryData;
// Stores telemetry string sent via BLE or Telnet.

uint8_t BatteryPercentage;
// Battery charge percentage.

uint64_t ChipID = ESP.getEfuseMac();  
// Unique chip ID (MAC address) for BLE and system identification.

// Web Server for OTA updates (ElegantOTA)
AsyncWebServer server(80);

// Telnet Server setup on port 23
WiFiServer telnetServer(23);
WiFiClient telnetClient;

// BLE Service UUIDs
BLEUUID ANEMOMETERSERVICE_UUID = BLEUUID((uint16_t)0x180A);
BLEUUID BATTERYSERVICE_UUID = BLEUUID((uint16_t)0x180F);

// BLE Characteristics
BLECharacteristic SerialNumberCharacteristic((uint16_t)0x2A25, BLECharacteristic::PROPERTY_READ);
// BLE characteristic for device serial number (read-only).

BLECharacteristic TelemetryCharacteristic(TELEMETRY_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
// Characteristic sending live telemetry data (wind direction + speed).

BLECharacteristic BatteryCharacteristic((uint16_t)0x2A19, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
// Characteristic for battery level (read and notify).

// BLE Descriptor for telemetry
BLEDescriptor TelemetryDescriptor = BLEDescriptor((uint16_t)0x2912, 200);
// Descriptor with maximum length of 200 bytes to describe telemetry data.


//------------------------------//
//           FUNCTIONS          //
//------------------------------//

void publishToTelnet(String message) {
  if (telnetClient && telnetClient.connected()) {
    telnetClient.print(message);       // Send message to Telnet client
  }
  Serial.print(message);               // Also print to serial monitor
}

// Converts analog reading from wind vane into a direction label (N, E, SW, etc.)
String GetWindDirection() {

  WindDirection[j] = analogRead(WIND_DIRECTION);
  // Read current analog value from wind direction sensor.

  publishToTelnet("WIND DIRECTION. Iteraton n°: " + String(j) + ", analog reading: " + String(WindDirection[j]));

  if (j == (DIRECTION_SAMPLE_NUMBER - 1))
    FirstTime_Direction = LOW;         // After first cycle, disable “first time” flag.

  unsigned int SampleNumber;
  if (FirstTime_Direction)
    SampleNumber = j;                  // Use fewer samples initially
  else 
    SampleNumber = DIRECTION_SAMPLE_NUMBER - 1; // Once full, use fixed number of samples.

  j = (j + 1) % DIRECTION_SAMPLE_NUMBER; // Move to next position in circular buffer.

  unsigned int AverageWindDirection = 0;

  for(unsigned int k = 0; k <= SampleNumber; k ++)
    AverageWindDirection += WindDirection[k];  // Sum up readings.

  publishToTelnet(", total sum: " + String(AverageWindDirection));

  AverageWindDirection /= (SampleNumber + 1);  // Compute average.

  publishToTelnet(", Average wind direction: " + String(AverageWindDirection) + "\n");

  // Map analog average to textual wind direction.
  if (AverageWindDirection < 800) 
    return "N";
  else if (AverageWindDirection < 1000)
    return "SW";
  else if (AverageWindDirection < 1100)
    return "NE";
  else if (AverageWindDirection < 1200)
    return "W";
  else if (AverageWindDirection < 1300)
    return "SE";
  else if (AverageWindDirection < 1500)
    return "N";
  else if (AverageWindDirection < 1900)
    return "E";
  else
    return "S";
}

// Calculates wind speed based on pulse count.
String GetWindSpeed(void) {

  currentTime = millis();  // Capture current time.

  noInterrupts();          // Temporarily disable interrupts to read stable count.
  count = pulseCount;      // Copy and reset pulse count.
  pulseCount = 0;
  interrupts();            // Re-enable interrupts.

  // Calculate wind speed (m/s). The “count” is doubled because ISR counts twice per revolution.
  WindSpeed[i] = PI * ANEMOMETER_RADIUS * 1000 * count / (currentTime - lastSampleTime);

  publishToTelnet("WIND SPEED. Iteration n°: " + String(i) + ", number of full spins: " + 
                  String(count) + " in " + String(currentTime - lastSampleTime) + 
                  " milliseconds at " + String(WindSpeed[i]) + " m/s");

  lastSampleTime = currentTime;  // Update time for next measurement.

  if (i == (SPEED_SAMPLE_NUMBER - 1))
    FirstTime_Speed = LOW;       // Mark completion of first averaging cycle.

  unsigned int SampleNumber;
  if (FirstTime_Speed)
    SampleNumber = i;
  else 
    SampleNumber = SPEED_SAMPLE_NUMBER - 1;

  i = (i + 1) % SPEED_SAMPLE_NUMBER;  // Circular buffer increment.

  float AverageWindSpeed = 0;

  for(unsigned int k = 0; k <= SampleNumber; k ++)
    AverageWindSpeed += WindSpeed[k]; // Sum up samples.

  publishToTelnet(", total sum: " + String(AverageWindSpeed));

  AverageWindSpeed /= (SampleNumber + 1);  // Compute average.

  publishToTelnet(", Average wind speed: " + String(AverageWindSpeed));

  return String(AverageWindSpeed);  // Return average as string.
}


//--- Interrupt Service Routine for wind speed pulses ---
void IRAM_ATTR countPulse() { // Store function in IRAM for faster access.
  lastInterruptTime = millis();
  if ((lastInterruptTime - firstInterruptTime)  > SAMPLE_TIME) {  // Debounce check.
    pulseCount++;                  // Increment pulse counter.
    firstInterruptTime = lastInterruptTime;
  }
}


//------------------------------//
//            SETUP             //
//------------------------------//

void setup() {
  Serial.begin(SERIAL_BAUD);                      // Start serial communication.
  delay(DELAY_AFTER_SERIAL_INITIALIZED);          // Allow time to stabilize serial output.

  pinMode(WIND_SPEED, INPUT);                     // Configure wind speed pin as input.
  attachInterrupt(digitalPinToInterrupt(WIND_SPEED), countPulse, RISING);
  // Attach ISR to wind speed sensor pin on rising edge.
  
  lastSampleTime = millis();                      // Initialize time reference.

  WiFi.mode(WIFI_STA);                            // Set WiFi mode (station).
  bool result = WiFi.softAP(SSID, PASSWORD);      // Start access point mode.

  if (result) 
    Serial.println("Access Point started successfully!");
  else
    Serial.println("Failed to start Access Point.");

  Serial.print("WiFi SSID: ");
  Serial.println(SSID);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  // Start Telnet server
  telnetServer.begin();
  telnetServer.setNoDelay(true);
  Serial.println("Telnet server started on port 23");

  // Start web server for OTA updates
  server.begin();
  ElegantOTA.begin(&server);   

  // Initialize BLE device
  BLEDevice::init(BLE_SERVER_NAME);

  // Set BLE MTU size to maximum (512 bytes)
  esp_err_t MTU_size = BLEDevice::setMTU(512);

  // Create BLE server and services
  BLEServer *pAnemometerServer = BLEDevice::createServer();

  BLEService *pAnemometerService = pAnemometerServer->createService(ANEMOMETERSERVICE_UUID, 30, 0);
  BLEService *pBatteryService = pAnemometerServer->createService(BATTERYSERVICE_UUID, 5, 0);

  // Add BLE characteristics
  pAnemometerService->addCharacteristic(&SerialNumberCharacteristic);
  SerialNumberCharacteristic.setValue(String(ChipID).c_str());

  pAnemometerService->addCharacteristic(&TelemetryCharacteristic);
  TelemetryDescriptor.setValue("Wind direction, Wind speed [m/s]");
  TelemetryCharacteristic.addDescriptor(&TelemetryDescriptor);

  pBatteryService->addCharacteristic(&BatteryCharacteristic);

  // Start BLE services
  pAnemometerService->start();
  pBatteryService->start();

  // Set up BLE advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(ANEMOMETERSERVICE_UUID);
  pAdvertising->addServiceUUID(BATTERYSERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // Improve iPhone connectivity
  pAdvertising->setMaxPreferred(0x12);
  pAdvertising->start();               // Begin BLE advertising

  publishToTelnet("Set-up finished, chip ID: " + String(ChipID));

  delay(DELAY_AFTER_SERIAL_INITIALIZED);          // Stabilization delay. Do NOT delete
}


//------------------------------//
//             LOOP             //
//------------------------------//

void loop() {

  ElegantOTA.loop();  // Handle OTA updates.

  // Check if new Telnet client is connecting.
  if (telnetServer.hasClient()) {
    if (!telnetClient.connected()) {
      if (telnetClient) telnetClient.stop();       // Close previous client.
      telnetClient = telnetServer.available();     // Accept new client.
      Serial.println("New Telnet client connected");
      telnetClient.println("Welcome to Weather Station Telnet!");
    } 
    else {
      // Reject additional clients if one is already connected.
      WiFiClient tempClient = telnetServer.available();
      tempClient.println("Another client is already connected.");
      tempClient.stop();
    }
  }

  // Measure and log battery voltage
  VoltageSensing = ((analogRead(BATTERY_VOLTAGE) * ADC_POWER / ADC_RESOLUTION) + ADC_OFFET);
  BatteryLevel = 2.05 * VoltageSensing;  // Compensated battery voltage
  BatteryPercentage = 100 * (BatteryLevel - MIN_BATTERY_VOLTAGE) / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE);
  publishToTelnet("\nBATTERY. Voltage sensing: " + String(VoltageSensing) +
                  " [V], Voltage battery: " + String(BatteryLevel) +
                  " [V] Voltage percentage: " + String(BatteryPercentage) + " %\n");

  BatteryCharacteristic.setValue(&BatteryPercentage, 1);
  BatteryCharacteristic.notify();       // Send BLE update.

  // Collect and transmit wind data
  TelemetryData = GetWindDirection() + ", " + GetWindSpeed();
  publishToTelnet("\nNEW READING: " + TelemetryData + " m/s\n");

  TelemetryCharacteristic.setValue(TelemetryData.c_str());
  TelemetryCharacteristic.notify();     // Notify BLE clients.

  // Manage WiFi connection timeout logic
  if (WiFi.getMode() != WIFI_OFF) {
    if (NotBack) {
      if (WiFi.softAPgetStationNum() == 0) {
        StartTime_WiFiTimeout = millis();
        publishToTelnet("No WiFi clients connected!\n");
        NotBack = LOW;
      }
    }
    else {
      if ((millis() - StartTime_WiFiTimeout) > WIFI_TIMEOUT) {
        publishToTelnet("WiFi access point stopped!\n");
        WiFi.softAPdisconnect(HIGH);
        WiFi.mode(WIFI_OFF);
        Serial.println("WiFi OFF!");
      }
      else if (WiFi.softAPgetStationNum() != 0) {
        publishToTelnet("New WiFi client connected!");
        NotBack = HIGH;
      }
    }
  }

  delay(LOOP_SLEEP_TIME);  // Wait before next measurement.
}
