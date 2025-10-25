#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "wled.h"

#define I2CDEV_IMPLEMENTATION I2CDEV_ARDUINO_WIRE

#undef DEBUG_PRINT
#undef DEBUG_PRINTLN
#undef DEBUG_PRINTF

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#undef DEBUG_PRINT
#undef DEBUG_PRINTLN
#undef DEBUG_PRINTF

#ifdef WLED_DEBUG
  #define DEBUG_PRINT(x) DEBUGOUT(x)
  #define DEBUG_PRINTLN(x) DEBUGOUTLN(x)
  #define DEBUG_PRINTF(x...) DEBUGOUTF(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(x...)
#endif

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    // Wire included in wled.h
#endif

// Interrupt detection
volatile bool mpuInterrupt = false;
void IRAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

/*
 * Usermod High-Five Detector
 * 
 * Wykrywa "zbicie piątki" używając:
 * - MPU6050 (akcelerometr) - wykrywa uderzenie/przyspieszenie
 * - FSR (czujnik nacisku) - potwierdza fizyczny kontakt
 * 
 * Tylko gdy oba czujniki wykryją zdarzenie, aktywuje się preset
 */

class FsrTriggerUsermod : public Usermod {

  private:
    // MPU6050
    MPU6050 mpu;
    bool dmpReady = false;
    bool initDone = false;
    bool needsReinit = false;
    
    uint8_t mpuIntStatus;
    uint8_t devStatus;
    uint16_t packetSize;
    uint16_t fifoCount;
    uint8_t fifoBuffer[64];
    
    Quaternion qat;
    VectorInt16 aa;
    VectorInt16 gy;
    VectorInt16 aaReal;
    VectorInt16 aaWorld;
    VectorFloat gravity;
    float ypr[3] = {0.0f};
    
    // Zmienne konfiguracyjne
    bool enabled = true;
    int8_t fsrPin = 15;
    uint16_t fsrThreshold = 500;
    
    // Progi dla wykrywania high-five
    float accelThreshold = 5000.0f;  // Próg przyspieszenia (surowe wartości)
    float impactDecayTime = 750;      // Czas na wykrycie FSR po uderzeniu (ms)
    uint32_t lightDuration = 5000;    // Czas świecenia po wykryciu
    uint32_t cooldownTime = 1000;     // Czas blokady po wykryciu (zapobiega wielokrotnemu triggerowaniu)
    
    uint8_t activePreset = 11;
    uint8_t inactivePreset = 10;
    
    // Zmienne stanu
    bool ledsActive = false;
    uint32_t activationTime = 0;
    uint32_t lastImpactTime = 0;
    uint32_t lastTriggerTime = 0;
    bool impactDetected = false;
    bool wasPressed = false;
    
    float maxAccelMagnitude = 0.0f;  // Do debugowania

    int8_t mpuIntPin = -1; // konfigurowalny pin przerwania MPU6050

    bool pendingApply = false;
    uint8_t pendingPresetId = 0;
    uint32_t pendingApplyAt = 0;

    // FSR debouncing
    const uint8_t FSR_SAMPLES = 5;
    const uint16_t FSR_SAMPLE_INTERVAL_MS = 6; // 5*6=30ms total
    // minimalny odstęp między detekcjami
    uint32_t lastImpactTimeSafe = 0;
    
    // Nazwy dla konfiguracji
    static const char _name[];
    static const char _enabled[];
    static const char _fsrPin[];
    static const char _fsrThreshold[];
    static const char _accelThreshold[];
    static const char _impactDecay[];
    static const char _duration[];
    static const char _cooldown[];
    static const char _activePreset[];
    static const char _inactivePreset[];
    static const char _mpuIntPin[];

    // Sprawdza, czy FSR jest wciśnięty
    bool isFsrPressed() {
      if (fsrPin < 0) return false;
      int value = analogRead(fsrPin);
      return value > fsrThreshold;
    }

    // Oblicza wielkość wektora przyspieszenia
    float getAccelMagnitude() {
      float mag = sqrt(aaWorld.x * aaWorld.x + 
                      aaWorld.y * aaWorld.y + 
                      aaWorld.z * aaWorld.z);
      if (mag > maxAccelMagnitude) {
        maxAccelMagnitude = mag;
      }
      return mag;
    }

    void readDmp() {
      if (!dmpReady) return;

      // jeśli mamy przerwanie sprzętowe — działamy tylko na ISR (mniejsze false positives)
      #if defined(MPU6050_INT_GPIO)
        if (mpuIntPin >= 0) {
          if (!mpuInterrupt) return;
          mpuInterrupt = false;
        }
      #endif

      // pobierz licznik FIFO i obsłuż overflow
      uint16_t fifoCount = mpu.getFIFOCount();
      if (fifoCount == 1024) { // overflow
        USER_PRINTLN(F("MPU FIFO overflow! resetting FIFO"));
        mpu.resetFIFO();
        return;
      }

      // jeśli brak wystarczającego danych, a działamy w trybie polling, po prostu zwracamy
      if (fifoCount < packetSize) return;

      // odczyt jednego pakietu
      while (fifoCount >= packetSize) {
        if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
          // niepełny pakiet - przerwij pętlę
          break;
        }

        mpu.dmpGetQuaternion(&qat, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &qat);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &qat);
        mpu.dmpGetYawPitchRoll(ypr, &qat, &gravity);

        // odśwież licznik i zwolnij FIFO jeśli trzeba
        fifoCount = mpu.getFIFOCount();
      }
    }


    void scheduleApplyPreset(uint8_t presetId) {
      // unikamy spamowania
      uint32_t now = millis();
      if (now - lastTriggerTime < cooldownTime) return;
      lastTriggerTime = now;

      pendingPresetId = presetId;
      pendingApply = true;
      pendingApplyAt = now + 5; // drobne opóźnienie, aby wszelkie ISR/wyścigi zdążyły się zakończyć
    }

    void performPendingApply() {
      if (!pendingApply) return;
      if (millis() < pendingApplyAt) return;
      pendingApply = false;

      // safety checks przed wywołaniem WLED API
      if (pendingPresetId == 0) return;

      // applyPreset może korzystać z kolejek; wywołujemy tylko z taska (to jest loop)
      applyPreset(pendingPresetId, CALL_MODE_DIRECT_CHANGE);

      // bezpieczne MQTT — sprawdź wskaźniki
      #ifndef WLED_DISABLE_MQTT
      if (WLED_MQTT_CONNECTED && mqtt != nullptr) {
        char subuf[64];
        strcpy(subuf, mqttDeviceTopic);
        strcat_P(subuf, PSTR("/highfive"));
        mqtt->publish(subuf, 0, false, "highfive");
      }
      #endif

      USER_PRINTLN(F("HIGH-FIVE SCHEDULED/EXECUTED"));
    }

    bool isFsrPressedDebounced() {
      if (fsrPin < 0) return false;
      uint16_t samples[FSR_SAMPLES];
      for (uint8_t i=0;i<FSR_SAMPLES;i++) {
        samples[i] = analogRead(fsrPin);
        delay(FSR_SAMPLE_INTERVAL_MS);
      }
      // prosty sort + wybór mediany (FSR_SAMPLES nie musi być duże)
      for (uint8_t i=0;i<FSR_SAMPLES-1;i++)
        for (uint8_t j=i+1;j<FSR_SAMPLES;j++)
          if (samples[j] < samples[i]) {
            uint16_t t = samples[i]; samples[i] = samples[j]; samples[j] = t;
          }
      uint16_t med = samples[FSR_SAMPLES/2];
      return med > fsrThreshold;
    }

    // DELAY-FREE helper: próbka jednorazowa (bez opóźnienia), do debugu
    int readFsrOnce() {
      if (fsrPin < 0) return 0;
      return analogRead(fsrPin);
    }

    // Wykrywa uderzenie na podstawie akcelerometru
    bool detectImpact() {
      if (!dmpReady) return false;
      
      float accelMag = getAccelMagnitude();
      return accelMag > accelThreshold;
    }

    // Aktywuje preset po wykryciu high-five
    void activateHighFive() {
      if (ledsActive) return;
      
      if (activePreset > 0) {
        applyPreset(activePreset, CALL_MODE_DIRECT_CHANGE);
      }
      
      ledsActive = true;
      activationTime = millis();
      lastTriggerTime = millis();
      
      USER_PRINTLN(F("HIGH-FIVE DETECTED!"));
      
      #ifndef WLED_DISABLE_MQTT
      publishMqtt("highfive");
      #endif
    }

    // Dezaktywuje preset
    void deactivatePreset() {
      if (!ledsActive) return;
      
      if (inactivePreset > 0) {
        applyPreset(inactivePreset, CALL_MODE_DIRECT_CHANGE);
      }
      
      ledsActive = false;
      impactDetected = false;
      
      #ifndef WLED_DISABLE_MQTT
      publishMqtt("idle");
      #endif
    }

    #ifndef WLED_DISABLE_MQTT
    void publishMqtt(const char* state) {
      if (WLED_MQTT_CONNECTED) {
        char subuf[64];
        strcpy(subuf, mqttDeviceTopic);
        strcat_P(subuf, PSTR("/highfive"));
        mqtt->publish(subuf, 0, false, state);
      }
    }
    #endif

    void setupMPU6050() {
      USER_PRINTLN(F("High-Five Detector: Initializing MPU6050..."));
      
      PinManagerPinType pins[2] = { { i2c_scl, true }, { i2c_sda, true } };
      
      if ((i2c_scl < 0) || (i2c_sda < 0)) {
        USER_PRINTF("High-Five: Invalid I2C pins: sda=%d scl=%d\n", i2c_sda, i2c_scl);
        return;
      }

      if (pins[1].pin < 0 || pins[0].pin < 0) {
        enabled = false;
        dmpReady = false;
        return;
      }

      if (!pinManager.joinWire()) {
        enabled = false;
        dmpReady = false;
        USER_PRINTF("High-Five: Failed to allocate I2C sda=%d scl=%d\n", i2c_sda, i2c_scl);
        return;
      }

      #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.setClock(100000);
      #endif

      // Inicjalizacja przerwania
      if ((mpuIntPin >= 0) && (pinManager.getPinOwner(mpuIntPin) != PinOwner::UM_IMU) 
         && !pinManager.allocatePin(mpuIntPin, false, PinOwner::UM_IMU)) {
        USER_PRINTF("High-Five: Warning - failed to allocate interrupt GPIO %d\n", mpuIntPin);
      }

      mpu.initialize();
      if (mpuIntPin >= 0) {
        pinMode(mpuIntPin, INPUT);
      }

      USER_PRINTLN(mpu.testConnection() ? F("MPU6050 connected") : F("MPU6050 connection failed"));

      devStatus = mpu.dmpInitialize();

      // Offsety gyro (można skalibrować dla własnego sensora)
      mpu.setXGyroOffset(220);
      mpu.setYGyroOffset(76);
      mpu.setZGyroOffset(-85);
      mpu.setZAccelOffset(1788);

      if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        
        mpu.setDMPEnabled(true);

        if (mpuIntPin >= 0) {
          attachInterrupt(digitalPinToInterrupt(mpuIntPin), dmpDataReady, RISING);
        }
        
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        USER_PRINTLN(F("MPU6050 DMP ready!"));
        mpu.resetFIFO();
      } else {
        USER_PRINTF("DMP Initialization failed (code %d)\n", devStatus);
        dmpReady = false;
      }
    }

  public:

    void setup() override {
      // Setup FSR
      if (fsrPin >= 0) {
        pinMode(fsrPin, INPUT);
      }
      
      // Setup MPU6050
      setupMPU6050();
      
      // Ustaw nieaktywny preset przy starcie
      if (inactivePreset > 0) {
        applyPreset(inactivePreset, CALL_MODE_DIRECT_CHANGE);
      }
      
      initDone = true;
    }

    void loop() override {
      if (!enabled || !initDone) return;

      if (needsReinit) {
        USER_PRINTLN(F("High-Five: applying updated configuration..."));
        needsReinit = false;

        // bezpieczna ponowna inicjalizacja czujników
        setupMPU6050();
        if (inactivePreset > 0) {
          applyPreset(inactivePreset, CALL_MODE_DIRECT_CHANGE);
        }
      }

      uint32_t now = millis();

      // 1) najpierw próbujemy bezpiecznie przeczytać dane z DMP (tylko gdy przerwanie)
      readDmp();

      // 2) wykonaj zaplanowane apply (jeśli był)
      performPendingApply();

      // 3) cooldown globalny
      if (now - lastTriggerTime < cooldownTime) {
        // nadal obsługujemy wygaśnięcie presetów
        if (ledsActive && (now - activationTime >= lightDuration)) {
          deactivatePreset();
        }
        return;
      }

      // 4) detekcja uderzenia — nieco bezpieczniejsza: threshold + dodatni pik i hysteresis
      if (dmpReady) {
        float accelMag = getAccelMagnitude(); // opiera się na aaWorld
        // debug rzadziej — tylko co 200ms
        static uint32_t lastDbg = 0;
        if (now - lastDbg > 200) {
          lastDbg = now;
          //USER_PRINTF("Accel: %.1f max: %.1f\n", accelMag, maxAccelMagnitude);
        }

        // prosty warunek: acceleracja musi przekroczyć threshold *i* być większa niż ostatniego piku
        if (!impactDetected && accelMag > accelThreshold && accelMag > (maxAccelMagnitude * 0.5f)) {
          impactDetected = true;
          lastImpactTime = now;
          USER_PRINTF("Impact detected! Accel: %.1f\n", accelMag);
        }
      }

      // 5) jeśli wykryto impact — czekaj krótko na FSR z debounce
      if (impactDetected) {
        // sprawdzamy presję FSR (debounced)
        bool pressed = isFsrPressedDebounced();

        if (pressed && !wasPressed) {
          // dodatkowe zabezpieczenie przed spamowaniem
          if (now - lastImpactTimeSafe > cooldownTime) {
            lastImpactTimeSafe = now;
            // planujemy apply (nie wywołujemy bezpośrednio)
            scheduleApplyPreset(activePreset);
            ledsActive = true;
            activationTime = now;
          }
          impactDetected = false;
        }
        wasPressed = pressed;

        if (now - lastImpactTime > impactDecayTime) {
          impactDetected = false;
          USER_PRINTLN(F("Impact timeout - no FSR confirmation"));
        }
      }

      // 6) auto-off
      if (ledsActive && (now - activationTime >= lightDuration)) {
        deactivatePreset();
      }
    }

    void addToJsonInfo(JsonObject& root) override {
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");

      JsonArray arr = user.createNestedArray(FPSTR(_name));
      
      if (enabled && dmpReady) {
        arr.add(ledsActive ? F("ACTIVE") : F("Ready"));
        
        // Debug info
        if (fsrPin >= 0) {
          int fsrValue = analogRead(fsrPin);
          arr.add(String(F("FSR: ")) + String(fsrValue));
        }
        
        float accelMag = getAccelMagnitude();
        arr.add(String(F("Accel: ")) + String((int)accelMag) + "/" + String((int)accelThreshold));
        
        if (impactDetected) {
          arr.add(F("⚡ Impact!"));
        }
      } else {
        arr.add(F("Disabled/Not Ready"));
      }
    }

    void addToJsonState(JsonObject& root) override {
      if (!initDone || !enabled) return;

      JsonObject usermod = root[FPSTR(_name)];
      if (usermod.isNull()) usermod = root.createNestedObject(FPSTR(_name));

      usermod["active"] = ledsActive;
      usermod["dmpReady"] = dmpReady;
      usermod["impactDetected"] = impactDetected;
      
      if (fsrPin >= 0) {
        usermod["fsrValue"] = analogRead(fsrPin);
      }
      
      if (dmpReady) {
        usermod["accelMagnitude"] = (int)getAccelMagnitude();
        usermod["maxAccel"] = (int)maxAccelMagnitude;
      }
      
      if (ledsActive) {
        uint32_t remaining = lightDuration - (millis() - activationTime);
        usermod["remaining"] = remaining;
      }
    }

    void addToConfig(JsonObject& root) override {
      JsonObject top = root.createNestedObject(FPSTR(_name));
      top[FPSTR(_enabled)] = enabled;
      top[FPSTR(_fsrPin)] = fsrPin;
      top[FPSTR(_fsrThreshold)] = fsrThreshold;
      top[FPSTR(_accelThreshold)] = accelThreshold;
      top[FPSTR(_impactDecay)] = impactDecayTime;
      top[FPSTR(_duration)] = lightDuration;
      top[FPSTR(_cooldown)] = cooldownTime;
      top[FPSTR(_activePreset)] = activePreset;
      top[FPSTR(_inactivePreset)] = inactivePreset;
      top[FPSTR(_mpuIntPin)] = mpuIntPin;
    }

    bool readFromConfig(JsonObject& root) override {
      JsonObject top = root[FPSTR(_name)];
      bool configComplete = !top.isNull();

      configComplete &= getJsonValue(top[FPSTR(_enabled)], enabled, true);
      configComplete &= getJsonValue(top[FPSTR(_fsrPin)], fsrPin, 15);
      configComplete &= getJsonValue(top[FPSTR(_mpuIntPin)], mpuIntPin, -1);
      configComplete &= getJsonValue(top[FPSTR(_fsrThreshold)], fsrThreshold, 500);
      configComplete &= getJsonValue(top[FPSTR(_accelThreshold)], accelThreshold, 15000.0f);
      configComplete &= getJsonValue(top[FPSTR(_impactDecay)], impactDecayTime, 200);
      configComplete &= getJsonValue(top[FPSTR(_duration)], lightDuration, 5000);
      configComplete &= getJsonValue(top[FPSTR(_cooldown)], cooldownTime, 1000);
      configComplete &= getJsonValue(top[FPSTR(_activePreset)], activePreset, 1);
      configComplete &= getJsonValue(top[FPSTR(_inactivePreset)], inactivePreset, 2);

      if (initDone && enabled) {
        needsReinit = true; // opóźniona reinicjalizacja z loop()
      }

      return configComplete;
    }

    void appendConfigData() override {
      oappend(SET_F("addInfo('High-Five:pin', 1, 'Pin FSR (analogowy, np. 15)');"));
      oappend(SET_F("addInfo('High-Five:mpuIntPin', 1, 'Pin przerwania MPU6050 (lub -1 dla polling)');"));
      oappend(SET_F("addInfo('High-Five:fsrThreshold', 1, 'Próg FSR (0-4095, def: 500)');"));
      oappend(SET_F("addInfo('High-Five:accelThreshold', 1, 'Próg uderzenia (def: 15000)');"));
      oappend(SET_F("addInfo('High-Five:impactDecay', 1, 'Okno czasowe FSR po uderzeniu (ms, def: 200)');"));
      oappend(SET_F("addInfo('High-Five:duration', 1, 'Czas świecenia (ms, def: 5000)');"));
      oappend(SET_F("addInfo('High-Five:cooldown', 1, 'Czas blokady po wykryciu (ms, def: 1000)');"));
      oappend(SET_F("addInfo('High-Five:activePreset', 1, 'ID presetu aktywnego');"));
      oappend(SET_F("addInfo('High-Five:inactivePreset', 1, 'ID presetu nieaktywnego');"));
    }

    uint16_t getId() override {
      return USERMOD_ID_FSR_TRIGGER;
    }
};

// Definicje stałych
const char FsrTriggerUsermod::_name[]              PROGMEM = "High-Five";
const char FsrTriggerUsermod::_enabled[]           PROGMEM = "enabled";
const char FsrTriggerUsermod::_fsrPin[]            PROGMEM = "pin";
const char FsrTriggerUsermod::_fsrThreshold[]      PROGMEM = "fsrThreshold";
const char FsrTriggerUsermod::_accelThreshold[]    PROGMEM = "accelThreshold";
const char FsrTriggerUsermod::_impactDecay[]       PROGMEM = "impactDecay";
const char FsrTriggerUsermod::_duration[]          PROGMEM = "duration";
const char FsrTriggerUsermod::_cooldown[]          PROGMEM = "cooldown";
const char FsrTriggerUsermod::_activePreset[]      PROGMEM = "activePreset";
const char FsrTriggerUsermod::_inactivePreset[]    PROGMEM = "inactivePreset";
const char FsrTriggerUsermod::_mpuIntPin[]         PROGMEM = "mpuIntPin";