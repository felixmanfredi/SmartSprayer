#include "utils.h"
#include "ble_callbacks.h"


// Logo bluetooth 8x12 pixel (bianco/nero)
const unsigned char bluetooth_logo [] PROGMEM = {
  0b00010000, //     |
  0b00010000, //     |
  0b00011000, //     |>
  0b00010100, //     | >
  0b10010010, // >   |  >
  0b01010100, //  >  | >
  0b00111000, //    >|>
  0b00110100, //    >| >
  0b01010010, //  >  |  >
  0b10010100, // >   | >
  0b00011000, //     |>
  0b00010000  //     |
};


// Icona batteria in carica 8x8 pixel (bianco/nero)
const unsigned char charge_icon [] PROGMEM = {
  0b00001000,
  0b00011000,
  0b00011000,
  0b00110000,
  0b01111100,
  0b00011000,
  0b00110000,
  0b00110000,
  0b00100000
};

// Lookup table per percentuale batteria LiPo 1S
static const struct { float v; int p; } LIPO_LUT[] = {
  {4.20,100},{4.15,95},{4.10,90},{4.05,85},{4.00,80},
  {3.95,75},{3.90,70},{3.85,65},{3.80,60},{3.75,55},
  {3.70,50},{3.65,45},{3.60,35},{3.55,25},{3.50,15},
  {3.45, 8},{3.40, 5},{3.35, 3},{3.30, 0}
};


// Variabili o oggetti esterni (definiti in main.cpp)
extern Adafruit_SSD1306 display;
extern bool bleConnected;

extern int16_t distance;
extern float temperature;
extern float humidity;
extern float pressure;

extern bool lasersOn;
extern bool systemActive; 


const float BATT_DIVIDER = 2.0f;

int rawPiezo = 0; 

// FFT setup
#define SAMPLES 512             
#define SAMPLING_RATE 2048
#define TARGET_FREQ 648.0f     
#define BANDWIDTH 30.0f
#define SOGLIA_ACCENSIONE 350.0  
#define SOGLIA_SPEGNIMENTO 300.0   


double vReal[SAMPLES];
double vImag[SAMPLES];

float maxMag;
unsigned long lastChangeTime = 0;

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_RATE);


// --- Funzioni ---

bool usbConnected() {
  return digitalRead(EXT_5V_SENSE) == HIGH;
}

float readBatteryVoltage() {
  uint32_t mv = analogReadMilliVolts(BATT_SENSE);
  float vbat = (mv / 1000.0f) * BATT_DIVIDER;
  return vbat;
}

uint8_t batteryPercentLipo(float vbat) {
  // Numero di elementi nella LUT
  const size_t n = sizeof(LIPO_LUT) / sizeof(LIPO_LUT[0]);

  if (vbat >= LIPO_LUT[0].v) return 100;
  if (vbat <= LIPO_LUT[n - 1].v) return 0;

  for (size_t i = 0; i < n - 1; ++i) {
    float v_hi = LIPO_LUT[i].v;
    float v_lo = LIPO_LUT[i + 1].v;

    if (vbat <= v_hi && vbat >= v_lo) {
      int p_hi = LIPO_LUT[i].p;
      int p_lo = LIPO_LUT[i + 1].p;

      // Interpolazione lineare tra i due punti
      float t = (vbat - v_lo) / (v_hi - v_lo);
      int p = (int)(p_lo + t * (p_hi - p_lo) + 0.5f);

      if (p < 0)   p = 0;
      if (p > 100) p = 100;

      Serial.printf("Vbat=%.2fV => %d%%\n", vbat, p);
      return (uint8_t)p;
    }
  }

  return 0; 
}


void drawBatteryIcon(int x, int y, int w, int h, int percent) {
  // Disegno il corpo batteria
  display.drawRect(x, y, w, h, WHITE);
  // Disegno il "polo" in alto
  display.fillRect(x + w, y + h/4, 3, h/2, WHITE);

  // Calcolo livello riempimento
  int fillWidth = map(percent, 0, 100, 0, w-4);
  display.fillRect(x+2, y+2, fillWidth, h-4, WHITE);


}


void drawInfoPage() {
  float vbat = readBatteryVoltage();
  int perc  = batteryPercentLipo(vbat);

  display.clearDisplay();

  // === Sezione in alto: stato batteria ===
  drawBatteryIcon(0, 0, 30, 15, perc);

  if (!usbConnected()) {
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(36, 2);
    display.printf("%3d%%", perc);
  } else {
    display.setTextSize(1);
    display.setTextColor(WHITE);
  }


  if (bleConnected)
    display.drawBitmap(110, 0, bluetooth_logo, 8, 12, WHITE);

  if (digitalRead(EXT_5V_SENSE) == HIGH)
    display.drawBitmap(36, 2, charge_icon, 8, 8, WHITE);

  display.setTextSize(1);
  display.setTextColor(WHITE);

  // display.setCursor(0, 20);
  // display.printf("dist: %d cm", distance);
  // display.setCursor(0, 32);
  // display.printf("T: %.1fC", temperature);
  // display.setCursor(0, 44); 
  // display.printf("H: %.1f%%", humidity);
  // display.setCursor(0, 56);
  // display.printf("P: %.2f hPa", pressure);
  display.setCursor(0, 20);
  display.printf("dist: %d cm", distance);
  display.setCursor(0, 32);
  display.printf("T: %.1fC", temperature);
  display.setCursor(60, 32);
  display.printf("H: %.1f%%", humidity);
  display.setCursor(0, 44);
  display.printf("P: %.2f hPa", pressure);
  display.setCursor(0, 56);
  display.printf("Vbat: %.2f V", vbat);
  display.setCursor(80, 56);
  display.printf("Sys: %s", systemActive ? "ON" : "OFF");

  display.display();
}



void drawLaserPage() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("[LASER]");

  display.setTextSize(1);
  display.setCursor(0, 20);
  if (lasersOn) {
    display.println("LASER ACCESI");
  } else {
    display.println("Premi il pulsante");
    display.println("per accendere");
  }

  display.display();
}


// Gestione dello stato del sistema (acceso/spento)

float analisiFFT() {
  // Acquisisce 512 campioni dal sensore piezo
  for(int i = 0; i < SAMPLES; i++) {
    vReal[i] = analogRead(PIEZO_SENSOR);
    vImag[i] = 0;
    delayMicroseconds(1000000 / SAMPLING_RATE);
  }

  // Calcola FFT
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // Trova magnitudo intorno a 648 Hz
  int targetBin = (int)(TARGET_FREQ * SAMPLES / SAMPLING_RATE);
  int binWidth = (int)(BANDWIDTH * SAMPLES / SAMPLING_RATE);

  double mag = 0;

  // Cerca il picco entro ±BANDWIDTH
  for(int i = targetBin - binWidth; i <= targetBin + binWidth; i++) {
    if(i >= 0 && i < SAMPLES/2) {
      if(vReal[i] > mag) {
        mag = vReal[i];
      }
    }
  }

  return (float)mag;
}


void updateSystemState(long now) {
  // Acquisisce magnitudo FFT a 648 Hz
  maxMag = analisiFFT();

  bool newState = systemActive;

  if (!systemActive && maxMag > SOGLIA_ACCENSIONE) {
    newState = true;
  } else if (systemActive && maxMag < SOGLIA_SPEGNIMENTO) {
    newState = false;
  }

  // Debounce: cambiamento solo dopo 200ms
  if (newState != systemActive) {
    if (now - lastChangeTime >= 200) {
      systemActive = newState;
      lastChangeTime = now;
      
      //Serial.printf("Sistema: %s (mag: %.1f)\n", systemActive ? "ACCESO" : "SPENTO", maxMag);
    }
  } else {
    // Stato stabile, resetta timer
    lastChangeTime = now;
  }
}



