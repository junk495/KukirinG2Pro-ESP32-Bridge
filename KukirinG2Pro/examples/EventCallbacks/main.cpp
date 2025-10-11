#include <Arduino.h>
#include "KukirinG2Pro.h"

// ============================================================================
// --- Configuration ---
// ============================================================================

// Set to true to enable all Serial Monitor output (debug info and events).
// Set to false for silent operation in a final project.
const bool ENABLE_SERIAL_OUTPUT = true;

// Set to true to simulate speed based on the throttle position.
// Set to false if you are providing real speed data from a sensor
// using the controller.setSpeedRaw() function.
const bool ENABLE_THROTTLE_SIMULATION = true;


// ============================================================================
// --- Hardware Setup ---
// ============================================================================

// Hardware configuration for ESP32 D1 Mini
#define DISPLAY_RX_PIN 17  // Connect to Display's TX (Green wire)
#define DISPLAY_TX_PIN 16  // Connect to Display's RX (Yellow wire)

KukirinG2Pro controller;

void setup() {
    if (ENABLE_SERIAL_OUTPUT) {
        Serial.begin(115200);
        delay(1000);
        Serial.println("\nKukirin G2 Pro - Callback Example");
    }

    // Initialize the controller library.
    // Serial2 is used for the communication with the display.
    // Pass 'Serial' as the second argument ONLY if debug output is enabled.
    if (ENABLE_SERIAL_OUTPUT) {
        controller.begin(Serial2, Serial);
    } else {
        controller.begin(Serial2);
    }

    // Enable throttle simulation if configured.
    if (ENABLE_THROTTLE_SIMULATION) {
        controller.enableThrottleSimulation(true);
        if (ENABLE_SERIAL_OUTPUT) {
            Serial.println("INFO: Speed simulation from throttle is ENABLED.");
        }
    } else {
        if (ENABLE_SERIAL_OUTPUT) {
            Serial.println("INFO: Speed simulation is DISABLED. Waiting for external sensor data via setSpeedRaw().");
        }
    }

    // --- Register Event Callback Functions ---
    // These functions will be called automatically by the library
    // whenever the corresponding state changes.

    controller.onLightChange([](bool isOn) {
        if (ENABLE_SERIAL_OUTPUT) {
            Serial.print(">>> EVENT: Light state changed to: ");
            Serial.println(isOn ? "ON" : "OFF");
        }
        // You could toggle a real light relay here.
    });

    controller.onBrakeChange([](bool isActive) {
        if (ENABLE_SERIAL_OUTPUT) {
            Serial.print(">>> EVENT: Brake state changed to: ");
            Serial.println(isActive ? "ACTIVE" : "INACTIVE");
        }
        // You could control a brake light here.
    });

    controller.onHornChange([](bool isActive) {
        if (ENABLE_SERIAL_OUTPUT) {
            Serial.print(">>> EVENT: Horn state changed to: ");
            Serial.println(isActive ? "ACTIVE" : "INACTIVE");
        }
    });
    
    controller.onBlinkerLeftChange([](bool isActive) {
        if (ENABLE_SERIAL_OUTPUT) {
            Serial.print(">>> EVENT: Blinker Left state changed to: ");
            Serial.println(isActive ? "ACTIVE" : "INACTIVE");
        }
    });

    controller.onBlinkerRightChange([](bool isActive) {
        if (ENABLE_SERIAL_OUTPUT) {
            Serial.print(">>> EVENT: Blinker Right state changed to: ");
            Serial.println(isActive ? "ACTIVE" : "INACTIVE");
        }
    });

    controller.onDriveModeChange([](uint8_t newMode) {
        if (ENABLE_SERIAL_OUTPUT) {
            Serial.print(">>> EVENT: Drive Mode changed to: L");
            if (newMode == 0x05) Serial.println("1");
            else if (newMode == 0x0A) Serial.println("2");
            else if (newMode == 0x0F) Serial.println("3");
            else Serial.println("?");
        }
    });

    controller.onThrottleChange([](uint16_t newThrottle) {
        if (ENABLE_SERIAL_OUTPUT) {
            // Optional: Only print if the value changes significantly
            // to avoid flooding the console.
            static uint16_t lastThrottle = 0;
            if (abs(newThrottle - lastThrottle) > 5) {
                Serial.print(">>> EVENT: Throttle changed to: ");
                Serial.println(newThrottle);
                lastThrottle = newThrottle;
            }
        }
    });
}

void loop() {
    // The update() function must be called continuously in the main loop.
    // It handles all communication and triggers the callbacks.
    controller.update();

    // If throttle simulation is DISABLED, you must provide speed data here.
    if (!ENABLE_THROTTLE_SIMULATION) {
        // --- Your Sensor Logic Goes Here ---
        // Example: Simulate a sensor that slowly accelerates and decelerates.
        // Replace this with your actual sensor reading code.
        float speed_kmh = 25.0 * (1.0 + sin(millis() / 2000.0)) / 2.0; // Oscillates between 0 and 25 km/h
        
        // Convert km/h to the required speedRaw value (simplified formula)
        uint16_t speedRawValue = 3500; // Default value for idle
        if (speed_kmh > 0.5) {
            // Note: For a real sensor, you would use the more precise formula
            // that includes wheel circumference and motor poles.
            speedRawValue = (uint16_t)(2550.0 / speed_kmh);
        }

        // Clamp the value to the valid range
        if (speedRawValue < 0x0030) speedRawValue = 0x0030;
        if (speedRawValue > 3500) speedRawValue = 3500;
        
        // Feed the sensor value to the library
        controller.setSpeedRaw(speedRawValue);
    }
}

