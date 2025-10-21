#include <Arduino.h>
#include "KukirinG2Pro.h"
#include "VescUart.h"

// ============================================================================
// --- Globale Konfiguration & Tuning-Parameter ---
// ============================================================================

// --- Geschwindigkeits-Limits für "Open" Modus ---
const float SPEED_LIMIT_OPEN_L1 = 7.0f;
const float SPEED_LIMIT_OPEN_L2 = 15.0f;
const float SPEED_LIMIT_OPEN_L3 = 25.0f;

// --- Geschwindigkeits-Limits für "Limited" Modus ---
const float SPEED_LIMIT_LIMITED_L1 = 20.0f;
const float SPEED_LIMIT_LIMITED_L2 = 30.0f;
const float SPEED_LIMIT_LIMITED_L3 = 45.0f;

// --- Beschleunigungs-Rampen-Steuerung (Änderung der eRPM pro Sekunde) ---
const float ACCEL_RAMP_RATES[5] = {
    10000.0f, // Stufe 1 (PA=1): Sehr sanft
    20000.0f, // Stufe 2 (PA=2): Sanft
    40000.0f, // Stufe 3 (PA=3): Normal
    80000.0f, // Stufe 4 (PA=4): Sportlich
    500000.0f // Stufe 5 (PA=5): Direkt (nahezu keine Rampe)
};
const float DECCEL_RAMP_RATE = 80000.0f;

// --- Bremsstrom-Stufen (Rekuperation in Ampere) ---
const float BRAKE_CURRENT_RATES[6] = {
    0.0f,   // Stufe 0 (PB=0): Aus
    -1.0f,  // Stufe 1 (PB=1): Sehr leicht
    -2.5f,  // Stufe 2 (PB=2): Leicht
    -5.0f,  // Stufe 3 (PB=3): Mittel
    -7.5f,  // Stufe 4 (PB=4): Stark
    -10.0f  // Stufe 5 (PB=5): Maximal
};

// --- Timing für Blinker & Bremslicht ---
const unsigned long BLINKER_INTERVAL_MS = 333;      // 90 Zyklen/Minute -> 1 Zyklus = 666ms -> halber Zyklus = 333ms
const unsigned long BRAKE_BLINK_INTERVAL_MS = 250;  // 120 Zyklen/Minute -> 1 Zyklus = 500ms -> halber Zyklus = 250ms

// --- Debug-Ausgaben ---
const bool ENABLE_SERIAL_OUTPUT = true;
const bool ENABLE_KUKIRIN_LIB_DEBUG = false;

// --- Betriebsmodus ---
const bool ENABLE_THROTTLE_SIMULATION = false;

// --- VESC Konfiguration ---
const uint32_t VESC_BAUD_RATE = 115200;
const bool ENABLE_VESC_LIB_DEBUG = false;

// --- Timing ---
const unsigned long VESC_UPDATE_INTERVAL_MS = 50;
const unsigned long VESC_COMMAND_INTERVAL_MS = 20;
const unsigned long VESC_CONNECTION_TIMEOUT_MS = 1000;
const float MIN_ERPM_FOR_REGEN = 100.0f; // Schwellenwert, ab dem regeneratives Bremsen aktiv wird


// ============================================================================
// --- Hardware-Setup ---
// ============================================================================

// --- Serielle Kommunikation ---
#define DISPLAY_RX_PIN 17
#define DISPLAY_TX_PIN 16
#define VESC_RX_PIN    19
#define VESC_TX_PIN    18

// --- Pin-Definitionen für Licht & Hupe ---
#define LIGHT_FRONT_PIN     23
#define LIGHT_REAR_PIN      22
#define HORN_PIN            21
#define BLINKER_LEFT_PIN    5
#define BLINKER_RIGHT_PIN   26 // Geändert von 4 auf 26


// ============================================================================
// --- Globale Objekte & Zustandsvariablen ---
// ============================================================================

KukirinG2Pro controller;
VescUart vesc;

// --- VESC-Zustand ---
bool g_isVescConnected = false;
unsigned long g_lastVescResponseTime = 0;
unsigned long g_lastVescUpdateTime = 0;
unsigned long g_lastVescCommandTime = 0;

// --- Fahr-Zustand ---
volatile uint16_t g_throttleValue = 0;
volatile uint8_t g_currentDriveMode = 0x05;
bool g_isSpeedProfileOpen = false; 
float g_currentSpeedKmh = 0.0f;
float g_currentErpmCommand = 0.0f;
uint8_t g_accelLevel = 3;
bool g_isBrakeActive = false;
uint8_t g_recupLevel = 3;

// --- Zustandsvariablen für Lichtsteuerung ---
bool g_isLightOn = false;
bool g_isBlinkerLeftOn = false;
bool g_isBlinkerRightOn = false;
unsigned long g_lastBlinkerToggleTime = 0;
bool g_blinkerPinState = false;
unsigned long g_lastBrakeLightToggleTime = 0;
bool g_brakeLightPinState = false;


// ============================================================================
// --- Funktions-Prototypen ---
// ============================================================================
void handleLights();


// ============================================================================
// --- Hilfsfunktionen für die Umrechnung ---
// ============================================================================

float calculateKmhFromErpm(float erpm) {
    uint16_t poleCount = controller.getPoleCount() > 0 ? controller.getPoleCount() : 30;
    float wheelDiameterInches = (float)controller.getWheelInches() / 10.0f;
    if (wheelDiameterInches < 1.0f) wheelDiameterInches = 9.0f;
    float polePairs = (float)poleCount / 2.0f;
    if (polePairs <= 0) return 0.0f;
    float mechanicalRpm = erpm / polePairs;
    float wheelDiameterM = wheelDiameterInches * 0.0254f;
    float wheelCircumferenceM = wheelDiameterM * PI;
    float speedMetersPerMinute = mechanicalRpm * wheelCircumferenceM;
    return (speedMetersPerMinute * 60.0f) / 1000.0f;
}

uint16_t calculateSpeedRawFromKmh(float kmh) {
    if (kmh < 0.5f) return 3500;
    uint16_t poleCount = controller.getPoleCount() > 0 ? controller.getPoleCount() : 30;
    uint8_t wheelInchesRaw = controller.getWheelInches() > 0 ? controller.getWheelInches() : 100;
    float pole_adj = 30.0f / (float)poleCount;
    float wheel_adj = 100.0f / (float)wheelInchesRaw;
    float adj_const = 2550.0f * pole_adj * wheel_adj;
    uint16_t speedRaw = (uint16_t)(adj_const / kmh);
    if (speedRaw < 0x0030) speedRaw = 0x0030;
    if (speedRaw > 3500) speedRaw = 3500;
    return speedRaw;
}

float calculateErpmFromKmh(float kmh) {
    uint16_t poleCount = controller.getPoleCount() > 0 ? controller.getPoleCount() : 30;
    float wheelDiameterInches = (float)controller.getWheelInches() / 10.0f;
    if (wheelDiameterInches < 1.0f) wheelDiameterInches = 9.0f;
    float polePairs = (float)poleCount / 2.0f;
    float wheelDiameterM = wheelDiameterInches * 0.0254f;
    float wheelCircumferenceM = wheelDiameterM * PI;
    float speedMetersPerMinute = (kmh * 1000.0f) / 60.0f;
    float mechanicalRpm = speedMetersPerMinute / wheelCircumferenceM;
    return mechanicalRpm * polePairs;
}


void setup() {
    if (ENABLE_SERIAL_OUTPUT) {
        Serial.begin(115200);
        delay(1000);
        Serial.println("\nKukirin G2 Pro - VESC Bridge (Stufe 7 - mit Licht & Hupe)");
    }

    pinMode(LIGHT_FRONT_PIN, OUTPUT);
    pinMode(LIGHT_REAR_PIN, OUTPUT);
    pinMode(HORN_PIN, OUTPUT);
    pinMode(BLINKER_LEFT_PIN, OUTPUT);
    pinMode(BLINKER_RIGHT_PIN, OUTPUT);

    if (ENABLE_SERIAL_OUTPUT) {
        controller.begin(Serial2, Serial);
        controller.enableDebugOutput(ENABLE_KUKIRIN_LIB_DEBUG);
    } else {
        controller.begin(Serial2);
    }
    
    Serial1.begin(VESC_BAUD_RATE, SERIAL_8N1, VESC_RX_PIN, VESC_TX_PIN);
    vesc.setSerialPort(&Serial1);
    if (ENABLE_SERIAL_OUTPUT && ENABLE_VESC_LIB_DEBUG) {
        vesc.setDebugPort(&Serial);
    }

    controller.enableThrottleSimulation(ENABLE_THROTTLE_SIMULATION);
    
    // --- Callbacks registrieren ---
    controller.onThrottleChange([](uint16_t newThrottle) {
        g_throttleValue = newThrottle;
    });

    controller.onDriveModeChange([](uint8_t newMode) {
        g_currentDriveMode = newMode;
        if (ENABLE_SERIAL_OUTPUT) {
            Serial.print(">>> EVENT: Drive Mode changed to: L");
            if (newMode == 0x05) Serial.println("1");
            else if (newMode == 0x0A) Serial.println("2");
            else if (newMode == 0x0F) Serial.println("3");
        }
    });
    
    controller.onBrakeChange([](bool isActive) {
        g_isBrakeActive = isActive;
    });

    controller.onLightChange([](bool isOn){
        g_isLightOn = isOn;
        if (ENABLE_SERIAL_OUTPUT) Serial.printf(">>> EVENT: Light is now %s\n", isOn ? "ON" : "OFF");
    });

    controller.onHornChange([](bool isActive){
        digitalWrite(HORN_PIN, isActive);
        if (ENABLE_SERIAL_OUTPUT) Serial.printf(">>> EVENT: Horn is now %s\n", isActive ? "ON" : "OFF");
    });

    controller.onBlinkerLeftChange([](bool isActive){
        g_isBlinkerLeftOn = isActive;
        if (ENABLE_SERIAL_OUTPUT) Serial.printf(">>> EVENT: Blinker Left is now %s\n", isActive ? "ON" : "OFF");
    });

    controller.onBlinkerRightChange([](bool isActive){
        g_isBlinkerRightOn = isActive;
        if (ENABLE_SERIAL_OUTPUT) Serial.printf(">>> EVENT: Blinker Right is now %s\n", isActive ? "ON" : "OFF");
    });

    controller.onSpeedProfileChange([](bool isOpen) {
        g_isSpeedProfileOpen = isOpen;
        if (ENABLE_SERIAL_OUTPUT) {
            Serial.print(">>> EVENT: Speed Profile changed to: ");
            Serial.println(isOpen ? "OPEN" : "LIMITED");
        }
    });

    controller.onRecupLevelChange([](uint8_t newLevel) {
        g_recupLevel = newLevel;
        if (ENABLE_SERIAL_OUTPUT) {
            Serial.printf(">>> EVENT: Recuperation Level (PB) changed to: %d\n", g_recupLevel);
        }
    });

    controller.onAccelLevelChange([](uint8_t newLevel) {
        g_accelLevel = newLevel;
        if (ENABLE_SERIAL_OUTPUT) {
            Serial.printf(">>> EVENT: Acceleration Level (PA) changed to: %d\n", g_accelLevel);
        }
    });
}

void loop() {
    controller.update();

    handleLights();

    if (millis() - g_lastVescUpdateTime > VESC_UPDATE_INTERVAL_MS) {
        g_lastVescUpdateTime = millis();
        if (vesc.getVescValues()) {
            if (!g_isVescConnected) g_isVescConnected = true;
            g_lastVescResponseTime = millis();
            g_currentSpeedKmh = calculateKmhFromErpm(vesc.data.rpm);
            controller.setSpeedRaw(calculateSpeedRawFromKmh(g_currentSpeedKmh));
        } else {
            if (g_isVescConnected) g_isVescConnected = false;
        }
    }

    if (millis() - g_lastVescCommandTime > VESC_COMMAND_INTERVAL_MS) {
        g_lastVescCommandTime = millis();
        if (g_isVescConnected && controller.getPoleCount() > 0) {
            if (g_isBrakeActive) {
                if (vesc.data.rpm > MIN_ERPM_FOR_REGEN) {
                    // Fahrzeug bewegt sich: Regeneratives Bremsen anwenden
                    float brakeCurrent = BRAKE_CURRENT_RATES[g_recupLevel];
                    vesc.setCurrent(brakeCurrent);
                    if (ENABLE_SERIAL_OUTPUT) {
                        if (g_recupLevel == 0) {
                            Serial.printf("PB%d BRAKE | OFF (RPM > %.0f)\n", g_recupLevel, MIN_ERPM_FOR_REGEN);
                        } else {
                            Serial.printf("PB%d BRAKE | REGEN: %.1fA (RPM=%.0f)\n", g_recupLevel, brakeCurrent, vesc.data.rpm);
                        }
                    }
                } else {
                    // Fahrzeug steht still: Nur 0A senden, um Rückwärtsdrehung zu verhindern
                    vesc.setCurrent(0.0f);
                    if (ENABLE_SERIAL_OUTPUT) {
                        Serial.printf("PB%d BRAKE | HOLD: 0.0A (RPM < %.0f)\n", g_recupLevel, MIN_ERPM_FOR_REGEN);
                    }
                }
                g_currentErpmCommand = vesc.data.rpm; 
            } else {
                uint16_t currentThrottle = g_throttleValue;
                if (currentThrottle < 70) {
                    currentThrottle = 0;
                }
                float maxSpeedKmhForMode = 0.0f;
                uint16_t maxThrottleForMode = 1000;
                if (g_isSpeedProfileOpen) {
                    switch (g_currentDriveMode) {
                        case 0x05: maxSpeedKmhForMode = SPEED_LIMIT_OPEN_L1; maxThrottleForMode = 360; break;
                        case 0x0A: maxSpeedKmhForMode = SPEED_LIMIT_OPEN_L2; maxThrottleForMode = 680; break;
                        case 0x0F: default: maxSpeedKmhForMode = SPEED_LIMIT_OPEN_L3; maxThrottleForMode = 1000; break;
                    }
                } else {
                    switch (g_currentDriveMode) {
                        case 0x05: maxSpeedKmhForMode = SPEED_LIMIT_LIMITED_L1; maxThrottleForMode = 360; break;
                        case 0x0A: maxSpeedKmhForMode = SPEED_LIMIT_LIMITED_L2; maxThrottleForMode = 680; break;
                        case 0x0F: default: maxSpeedKmhForMode = SPEED_LIMIT_LIMITED_L3; maxThrottleForMode = 1000; break;
                    }
                }
                float maxErpmForMode = calculateErpmFromKmh(maxSpeedKmhForMode);
                float targetErpm = map(currentThrottle, 0, maxThrottleForMode, 0, (long)maxErpmForMode);
                float rampRatePerSecond = ACCEL_RAMP_RATES[g_accelLevel - 1];
                float maxErpmChange = rampRatePerSecond * (VESC_COMMAND_INTERVAL_MS / 1000.0f);
                float maxDeccelChange = DECCEL_RAMP_RATE * (VESC_COMMAND_INTERVAL_MS / 1000.0f);
                if (targetErpm > g_currentErpmCommand) {
                    g_currentErpmCommand += maxErpmChange;
                    if (g_currentErpmCommand > targetErpm) g_currentErpmCommand = targetErpm;
                } else if (targetErpm < g_currentErpmCommand) {
                    g_currentErpmCommand -= maxDeccelChange;
                    if (g_currentErpmCommand < targetErpm) g_currentErpmCommand = targetErpm;
                }
                vesc.setRPM(g_currentErpmCommand);
                if (ENABLE_SERIAL_OUTPUT && currentThrottle > 0) {
                    Serial.printf("PA%d L%d %s | Thr: %d/%d -> eRPM: %.0f/%.0f -> VESC: %.0f (%.1f km/h)\n",
                        g_accelLevel, (g_currentDriveMode == 0x05) ? 1 : (g_currentDriveMode == 0x0A) ? 2 : 3,
                        g_isSpeedProfileOpen ? "OPEN " : "LIMIT", currentThrottle, maxThrottleForMode,
                        targetErpm, maxErpmForMode, g_currentErpmCommand, maxSpeedKmhForMode);
                } else if (currentThrottle == 0 && g_currentErpmCommand > 0) {
                    vesc.setRPM(0);
                    g_currentErpmCommand = 0;
                }
            }
        }
    }

    if (g_isVescConnected && (millis() - g_lastVescResponseTime > VESC_CONNECTION_TIMEOUT_MS)) {
        g_isVescConnected = false;
    }
}


// ============================================================================
// --- Hauptfunktion für die Lichtsteuerung ---
// ============================================================================
void handleLights() {
    unsigned long currentTime = millis();

    // --- Blinker-Logik ---
    if (g_isBlinkerLeftOn || g_isBlinkerRightOn) {
        if (currentTime - g_lastBlinkerToggleTime > BLINKER_INTERVAL_MS) {
            g_blinkerPinState = !g_blinkerPinState; // Zustand umschalten (AN/AUS)
            g_lastBlinkerToggleTime = currentTime;
        }
    } else {
        g_blinkerPinState = false; // Wenn kein Blinker aktiv ist, ist der Zustand immer AUS
    }
    digitalWrite(BLINKER_LEFT_PIN, g_isBlinkerLeftOn ? g_blinkerPinState : LOW);
    digitalWrite(BLINKER_RIGHT_PIN, g_isBlinkerRightOn ? g_blinkerPinState : LOW);

    // --- Frontlicht-Logik (einfach) ---
    digitalWrite(LIGHT_FRONT_PIN, g_isLightOn);

    // --- Rücklicht- & Bremslicht-Logik (komplex) ---
    if (g_isLightOn) {
        // Fall 1: Hauptlicht ist AN
        if (g_isBrakeActive) {
            // Bremslicht blinkt, wenn Hauptlicht an ist
            if (currentTime - g_lastBrakeLightToggleTime > BRAKE_BLINK_INTERVAL_MS) {
                g_brakeLightPinState = !g_brakeLightPinState;
                g_lastBrakeLightToggleTime = currentTime;
            }
            digitalWrite(LIGHT_REAR_PIN, g_brakeLightPinState);
        } else {
            // Rücklicht leuchtet dauerhaft, wenn Hauptlicht an ist und nicht gebremst wird
            digitalWrite(LIGHT_REAR_PIN, HIGH);
        }
    } else {
        // Fall 2: Hauptlicht ist AUS
        if (g_isBrakeActive) {
            // Bremslicht leuchtet dauerhaft, wenn Hauptlicht aus ist
            digitalWrite(LIGHT_REAR_PIN, HIGH);
        } else {
            // Rücklicht ist aus, wenn Hauptlicht aus ist und nicht gebremst wird
            digitalWrite(LIGHT_REAR_PIN, LOW);
        }
    }
}

