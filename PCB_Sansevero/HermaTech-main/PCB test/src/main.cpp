#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>

#define BATT_SENSE 1
#define BUZZER 3
#define SCL_I2C 4
#define SDA_I2C 5
#define EXT_LED 6
#define ONBOARD_LED 17
#define PIEZO_SENSOR 18
#define MUX_LIDAR 33
#define EXT_5V_SENSE 34
#define DT_ENCODER 35
#define CLK_ENCODER 36
#define PB_ENCODER 37
#define PB_SWITCH 38
#define LASER_1 45
#define LASER_2 46
#define CONF_LIDAR 47


int encoderPosCount = 0; 
int pinALast;  
int aVal;
bool bCW;

TFLI2C lidar;

void setup() {
  // Initialize serial (optional)
  Serial.begin(115200);
  Serial.println("ESP32-S3 LED Test");

  Wire.begin(SDA_I2C, SCL_I2C);

  // OUTPUT DIGITAL PIN SETTINGS
  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(LASER_1, OUTPUT);
  pinMode(LASER_2, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(EXT_LED, OUTPUT);
  pinMode(EXT_LED, OUTPUT);
  pinMode(CONF_LIDAR, OUTPUT);

  digitalWrite(CONF_LIDAR, LOW);


  // INPUT DIGITAL PIN SETTINGS
  pinMode(PB_SWITCH, INPUT);
  pinMode(PB_ENCODER, INPUT);
  pinMode(DT_ENCODER, INPUT);
  pinMode(CLK_ENCODER, INPUT);
  pinMode(EXT_5V_SENSE, INPUT);

  // INPUT ANALOG PIN SETTINGS
  analogSetPinAttenuation(BATT_SENSE, ADC_ATTENDB_MAX);
}

void loop() {

  int16_t dist; 
  bool ok;
  ok = lidar.getData(dist, 0x10);

  Serial.print("\nLettura LIDAR corretta?   ");
  Serial.println(ok);

  if (ok) {
    Serial.print("LIDAR distance:");
    Serial.print(dist);
    Serial.println(" [cm]");
  }

  ///////////////////////////////////////////////////
  int16_t batt_pos_voltage_sense_raw_value = analogRead(BATT_SENSE);
  float batt_pos_voltage_sense = float(batt_pos_voltage_sense_raw_value) / 4095.0 * 3.29;

  Serial.print("Battery + voltage sense raw value: ");
  Serial.print(batt_pos_voltage_sense_raw_value);
  Serial.print(", Battery + voltage sense: ");
  Serial.print(batt_pos_voltage_sense);
  Serial.print(" [V]");
  Serial.print(", Battery + voltage: ");
  Serial.print(batt_pos_voltage_sense * 1.9877);
  Serial.println(" [V]");
  
  /////////////////////////////////////////////////
  Serial.print("External 5V sense: ");
  Serial.println(digitalRead(EXT_5V_SENSE));

  if (!digitalRead(EXT_5V_SENSE)) {
    digitalWrite(ONBOARD_LED, HIGH);
  }
  else {
    digitalWrite(ONBOARD_LED, LOW);
  }

  //////////////////////////////////////
  int16_t piezo_sense_raw_value = analogRead(PIEZO_SENSOR);

  Serial.print("Piezo sense raw value: ");
  Serial.println(piezo_sense_raw_value);
  
  //////////////////////////////////////

  bool pb_switch = digitalRead(PB_SWITCH);

  if (pb_switch == 0) {
    Serial.print("PUSH BUTTON SWITCH  pressed");
    delay(100);
  }

  ///////////////////////////////////////// 
  bool pb_encoder = digitalRead(PB_ENCODER);

  if (pb_encoder == 0) {
    Serial.print("PUSH BUTTON ENCODER pressed");
    delay(100);
  }


  ////////////////////////////////////////////////
  digitalWrite(EXT_LED, HIGH);
  digitalWrite(LASER_1, HIGH);
  digitalWrite(LASER_2, HIGH);

  ///////////////////////////////////////////////
  // tone(BUZZER, 1000);
  // delay(200);
  // noTone(BUZZER);

  //////////////////////////////////////////////////////////
  aVal = digitalRead(CLK_ENCODER);
  if (aVal != pinALast) { 
    if (digitalRead(DT_ENCODER) != aVal) { //We're Rotating Clockwise  
      encoderPosCount ++;
      bCW = true;
    } 
    else {
      bCW = false;
      encoderPosCount--;
    }

    if (bCW)
      Serial.println ("Rotate Clockwise");   
    else
      Serial.println("Rotate Counterclockwise");
      Serial.print("Encoder Count: ");
      Serial.println(encoderPosCount);
      Serial.println();
    } 
  pinALast = aVal;

  ////////////////////////////////////////////////////

  delay(500);
} 

///////////////////////////////////////////////

/*#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define SCA 8
#define SCL 9

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
  Serial.begin(115200);

  Wire.begin(SCA, SCL);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display static text
  display.println("Hello, world!");
  display.display(); 
}

void loop() {
  Serial.println("Hllo world!");
  delay(500);
  
  // Display static text
  display.println("Hello, world!");
  display.display(); 
 
}*/