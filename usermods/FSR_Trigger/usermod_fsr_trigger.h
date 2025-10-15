#pragma once

#include "wled.h"

/*
 * Usermod FSR Trigger
 * 
 * Przełącza między dwoma presetami po wykryciu naciśnięcia czujnika FSR (Force Sensitive Resistor)
 * Aktywny preset świeci przez 5 sekund, po czym wraca do presetu nieaktywnego
 * 
 * Instalacja:
 * 1. Skopiuj ten plik do folderu ze sketche WLED
 * 2. Zarejestruj usermod dodając w usermods_list.cpp:
 *    - Na górze: #include "usermod_fsr_trigger.h"
 *    - Na dole: registerUsermod(new FsrTriggerUsermod());
 */

class FsrTriggerUsermod : public Usermod {

  private:
    // Zmienne konfiguracyjne
    bool enabled = true;
    int8_t fsrPin = 15;  // Domyślny pin (GPIO15)
    uint16_t fsrThreshold = 500;  // Próg naciśnięcia (0-4095 dla ESP32)
    uint32_t lightDuration = 5000;  // Czas świecenia w milisekundach (5 sekund)
    uint8_t activePreset = 1;  // Numer presetu do włączenia (aktywny stan)
    uint8_t inactivePreset = 2;  // Numer presetu do włączenia po zakończeniu (nieaktywny stan)
    
    // Zmienne stanu
    bool initDone = false;
    bool ledsActive = false;
    bool wasPressed = false;
    uint32_t activationTime = 0;
    
    // Nazwy dla konfiguracji
    static const char _name[];
    static const char _enabled[];
    static const char _fsrPin[];
    static const char _threshold[];
    static const char _duration[];
    static const char _activePreset[];
    static const char _inactivePreset[];

    // Sprawdza, czy FSR jest wciśnięty
    bool isFsrPressed() {
      if (fsrPin < 0) return false;
      
      int value = analogRead(fsrPin);
      return value > fsrThreshold;
    }

    // Przełącza na aktywny preset
    void activatePreset() {
      if (ledsActive) return;
      
      // Włącz aktywny preset
      if (activePreset > 0) {
        applyPreset(activePreset, CALL_MODE_DIRECT_CHANGE);
      }
      
      ledsActive = true;
      activationTime = millis();
      
      #ifndef WLED_DISABLE_MQTT
      publishMqtt("activated");
      #endif
    }

    // Przełącza na nieaktywny preset
    void deactivatePreset() {
      if (!ledsActive) return;
      
      // Włącz nieaktywny preset
      if (inactivePreset > 0) {
        applyPreset(inactivePreset, CALL_MODE_DIRECT_CHANGE);
      }
      
      ledsActive = false;
      
      #ifndef WLED_DISABLE_MQTT
      publishMqtt("deactivated");
      #endif
    }

    #ifndef WLED_DISABLE_MQTT
    void publishMqtt(const char* state) {
      if (WLED_MQTT_CONNECTED) {
        char subuf[64];
        strcpy(subuf, mqttDeviceTopic);
        strcat_P(subuf, PSTR("/fsr"));
        mqtt->publish(subuf, 0, false, state);
      }
    }
    #endif

  public:

    void setup() override {
      if (fsrPin >= 0) {
        pinMode(fsrPin, INPUT);
      }
      
      // Ustaw nieaktywny preset przy starcie
      if (inactivePreset > 0) {
        applyPreset(inactivePreset, CALL_MODE_DIRECT_CHANGE);
      }
      
      initDone = true;
    }

    void loop() override {
      if (!enabled || !initDone || fsrPin < 0) return;
      
      bool isPressed = isFsrPressed();
      
      // Wykryj naciśnięcie (zbocze narastające)
      if (isPressed && !wasPressed) {
        activatePreset();
      }
      
      wasPressed = isPressed;
      
      // Sprawdź, czy upłynął czas i przełącz na nieaktywny preset
      if (ledsActive && (millis() - activationTime >= lightDuration)) {
        deactivatePreset();
      }
    }

    void addToJsonInfo(JsonObject& root) override {
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");

      JsonArray fsrArr = user.createNestedArray(FPSTR(_name));
      
      if (fsrPin >= 0 && enabled) {
        int value = analogRead(fsrPin);
        fsrArr.add(value);
        fsrArr.add(ledsActive ? F(" (ACTIVE)") : F(" (idle)"));
      } else {
        fsrArr.add(enabled ? F("disabled - no pin") : F("disabled"));
      }
    }

    void addToJsonState(JsonObject& root) override {
      if (!initDone || !enabled) return;

      JsonObject usermod = root[FPSTR(_name)];
      if (usermod.isNull()) usermod = root.createNestedObject(FPSTR(_name));

      usermod["active"] = ledsActive;
      usermod["currentPreset"] = ledsActive ? activePreset : inactivePreset;
      if (ledsActive) {
        uint32_t remaining = lightDuration - (millis() - activationTime);
        usermod["remaining"] = remaining;
      }
    }

    void addToConfig(JsonObject& root) override {
      JsonObject top = root.createNestedObject(FPSTR(_name));
      top[FPSTR(_enabled)] = enabled;
      top[FPSTR(_fsrPin)] = fsrPin;
      top[FPSTR(_threshold)] = fsrThreshold;
      top[FPSTR(_duration)] = lightDuration;
      top[FPSTR(_activePreset)] = activePreset;
      top[FPSTR(_inactivePreset)] = inactivePreset;
    }

    bool readFromConfig(JsonObject& root) override {
      JsonObject top = root[FPSTR(_name)];
      bool configComplete = !top.isNull();

      configComplete &= getJsonValue(top[FPSTR(_enabled)], enabled, true);
      configComplete &= getJsonValue(top[FPSTR(_fsrPin)], fsrPin, 15);
      configComplete &= getJsonValue(top[FPSTR(_threshold)], fsrThreshold, 500);
      configComplete &= getJsonValue(top[FPSTR(_duration)], lightDuration, 5000);
      configComplete &= getJsonValue(top[FPSTR(_activePreset)], activePreset, 1);
      configComplete &= getJsonValue(top[FPSTR(_inactivePreset)], inactivePreset, 2);

      return configComplete;
    }

    void appendConfigData() override {
      oappend(SET_F("addInfo('FSR Trigger:pin', 1, 'Pin do czujnika FSR (analogowy)');"));
      oappend(SET_F("addInfo('FSR Trigger:threshold', 1, 'Próg wyzwolenia (0-4095)');"));
      oappend(SET_F("addInfo('FSR Trigger:duration', 1, 'Czas świecenia w ms (5000 = 5 sek)');"));
      oappend(SET_F("addInfo('FSR Trigger:activePreset', 1, 'ID presetu aktywnego (włączony)');"));
      oappend(SET_F("addInfo('FSR Trigger:inactivePreset', 1, 'ID presetu nieaktywnego (wyłączony)');"));
    }

    uint16_t getId() override {
      return USERMOD_ID_FSR_TRIGGER;
    }
};

// Definicje stałych tekstowych
const char FsrTriggerUsermod::_name[]            PROGMEM = "FSR Trigger";
const char FsrTriggerUsermod::_enabled[]         PROGMEM = "enabled";
const char FsrTriggerUsermod::_fsrPin[]          PROGMEM = "pin";
const char FsrTriggerUsermod::_threshold[]       PROGMEM = "threshold";
const char FsrTriggerUsermod::_duration[]        PROGMEM = "duration";
const char FsrTriggerUsermod::_activePreset[]    PROGMEM = "activePreset";
const char FsrTriggerUsermod::_inactivePreset[]  PROGMEM = "inactivePreset";