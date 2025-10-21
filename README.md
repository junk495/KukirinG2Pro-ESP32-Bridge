# Kukirin G2 Pro Controller Library for ESP32
This is an Arduino/PlatformIO library for emulating the motor controller of the Kukirin G2 Pro e-scooter on an ESP32, specifically the Wemos D1 Mini model.
The primary goal of this project is to fully reverse-engineer and replicate the UART communication protocol between the display and the controller. 
This allows for the creation of custom controllers, monitoring tools, or "man-in-the-middle" applications for tuning.
In the examples folder of the library is a complete code for a Spintend UBOX Alu lite VESC-Controller.
## Key Features
- Full Protocol Emulation: Accurately replicates the entire communication, including the dynamic speed checksum, eliminating all communication errors (E-006).
- Event-Driven: Use modern callback functions (onLightChange, onBrakeChange, etc.) to react to events instead of polling states in your main loop.
- Flexible Speed Input: Easily switch between simulating speed based on the throttle input or feeding in data from a real-world speed sensor.
- Lightweight & Robust: Designed to be stable and efficient, with robust error handling for reliable communication.
## Hardware
This library is optimized for the ESP32 D1 Mini (Wemos).

A key advantage of the ESP32 is its 5V-tolerant inputs, which means you do not need a logic level shifter to connect to the 5V logic of the scooter's display.
- Display TX (Green) → ESP32 GPIO 16 (RX2)
- Display RX (Yellow) → ESP32 GPIO 17 (TX2)
## Quick Start
- Place the library in the lib/ folder of your PlatformIO project.
- Include the library in your main sketch:
```cpp
#include <Arduino.h>
#include "KukirinG2Pro.h"

KukirinG2Pro controller;

void setup() {
    // Initialize the controller
    controller.begin(Serial2);

    // Optional: Enable speed simulation from throttle
    controller.enableThrottleSimulation(true);

    // Register your event callbacks here...
    controller.onLightChange([](bool isOn){
        // Your code here...
    });
}

void loop() {
    // This function must be called continuously
    controller.update();
}
```
## Examples & Documentation
- A complete, production-ready example sketch can be found in the examples/ directory.
- For a deep dive into the reverse-engineered protocol, including all packet structures and formulas, please see the [Full Protocol Documentation](https://github.com/junk495/Kukirin-G2-Pro---UART-Communication-Protocol).

LicenseThis project is licensed under the MIT License.
