#pragma once // Così il file viene incluso una sola volta
#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include "config.h"


// --- Dichiarazioni Funzioni ---
bool usbConnected();
float readBatteryVoltage();
uint8_t batteryPercentLipo(float vbat);
void drawBatteryIcon(int x, int y, int w, int h, int percent);
void drawInfoPage();
void drawLaserPage();
float analisiFFT ();
void updateSystemState(long now);