#pragma once
#include <Arduino.h>
#include "config.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h> // Per i descrittori di Client Characteristic Configuration (CCC) necessari per le notifiche

// Variabili o oggetti esterni (definiti in main.cpp)
extern bool bleConnected;
extern bool lasersOn;
extern BLECharacteristic CommandResponseCharacteristic;
extern BLECharacteristic ErrorCodeCharacteristic;

void sendSystemState(bool state);


// --- Classe callback per gestire gli eventi di connessione/disconnessione BLE ---
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    bleConnected = true;
    Serial.println("Telefono connesso via BLE");
  };

  void onDisconnect(BLEServer* pServer) override {
    bleConnected = false;
    Serial.println("Telefono disconnesso");

    // Riavvia la pubblicazione BLE per permettere nuove connessioni
    delay(500);
    BLEDevice::startAdvertising();
    Serial.println("Advertising riavviato — Pronto per una nuova connessione");

  }
};


// --- Classe callback per gestire i comandi ricevuti via BLE - Gestione laser ---
class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    std::string value = pChar->getValue();
    if (value.empty()) return;

    String cmd = String(value.c_str());
    cmd.toUpperCase(); // Normalizzo a maiuscole
    Serial.print("Comando ricevuto: ");
    Serial.println(cmd);

    if (cmd == "LASER ON") {
      lasersOn = true;
      digitalWrite(LASER_1, HIGH);
      digitalWrite(LASER_2, HIGH);
      CommandResponseCharacteristic.setValue("OK: LASER ON");
    } 
    else if (cmd == "LASER OFF") {
      lasersOn = false;
      digitalWrite(LASER_1, LOW);
      digitalWrite(LASER_2, LOW);
      CommandResponseCharacteristic.setValue("OK: LASER OFF");
    }
    else if (cmd == "LASER STATUS") {
      if (lasersOn) {
        CommandResponseCharacteristic.setValue("STATUS: LASER ON");
      }
      else {
        CommandResponseCharacteristic.setValue("STATUS: LASER OFF");
      }
    }
    else{
      CommandResponseCharacteristic.setValue("ERRORE: COMANDO NON RICONOSCIUTO");
      ErrorCodeCharacteristic.setValue(String("Comando non valido ricevuto via BLE").c_str());
      ErrorCodeCharacteristic.notify();
    }

    // Invia la risposta BLE
    CommandResponseCharacteristic.notify();

    Serial.print("Comando ricevuto via BLE: ");
    Serial.println(cmd);

    Serial.print("Risposta inviata via BLE: ");
    Serial.println((char*)CommandResponseCharacteristic.getValue().c_str());
  }
};
