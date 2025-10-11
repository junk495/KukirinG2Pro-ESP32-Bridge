#ifndef KUKIRING2PRO_H
#define KUKIRING2PRO_H

#include "Arduino.h"
#include <functional> // Required for std::function (callbacks)

// TX Packet (Display -> Controller, 20 Bytes)
// This structure holds the data received from the display.
struct __attribute__((packed)) TXPacket {
    uint8_t  startMarker;
    uint8_t  packetLength;
    uint8_t  commandType;
    uint8_t  subCommand;
    uint8_t  driveMode;
    uint8_t  functionBitmask;
    uint8_t  poleCount_L;
    uint8_t  poleCount_H;
    uint8_t  wheelCirc_L;
    uint8_t  wheelCirc_H;
    uint8_t  recupAccByte;
    uint8_t  reserved_0x0B;
    uint8_t  speedProfile_L;
    uint8_t  speedProfile_H;
    uint8_t  batteryConfig_L;
    uint8_t  batteryConfig_H;
    uint8_t  throttle_H;
    uint8_t  throttle_L;
    uint8_t  indicator_L;
    uint8_t  indicator_H;
};

// Holds decoded, human-readable settings from the TXPacket.
struct DecodedSettings {
    uint8_t rekupLevel;
    uint8_t accLevel;
    uint16_t poleCount;
    uint8_t wheelInches;
    uint8_t batteryVolts;
};

class KukirinG2Pro {
public:
    // --- Callback Type Definitions ---
    using StateChangeCallback = std::function<void(bool newState)>;
    using DriveModeChangeCallback = std::function<void(uint8_t newMode)>;
    using ThrottleChangeCallback = std::function<void(uint16_t newThrottle)>;

    KukirinG2Pro();

    // --- Setup ---
    void begin(HardwareSerial& port);
    void begin(HardwareSerial& port, Stream& debugPort);

    // --- Main Loop Function ---
    // Must be called repeatedly in your sketch's loop().
    // Returns true if a new packet was received and processed.
    bool update();

    // --- Speed Control ---
    // By default, the library expects external sensor data via this function.
    void setSpeedRaw(uint16_t speedRaw);
    
    // Call this in setup() to enable the internal speed simulation from throttle.
    void enableThrottleSimulation(bool enable = true);

    // --- State Getters ---
    uint16_t getThrottle();
    bool isBrakeActive();
    bool isLightOn();
    bool isHornOn();
    bool isBlinkerLeftOn();
    bool isBlinkerRightOn();
    const DecodedSettings& getSettings();
    const TXPacket& getRawTXPacket();

    // --- Event Callback Registration ---
    void onLightChange(StateChangeCallback callback);
    void onBrakeChange(StateChangeCallback callback);
    void onHornChange(StateChangeCallback callback);
    void onBlinkerLeftChange(StateChangeCallback callback);
    void onBlinkerRightChange(StateChangeCallback callback);
    void onDriveModeChange(DriveModeChangeCallback callback);
    void onThrottleChange(ThrottleChangeCallback callback);

private:
    // --- Private Methods ---
    void _handleDisplayPacket();
    void _sendResponse();
    uint8_t _calculateSpeedChecksum(uint8_t speed_H, uint8_t speed_L);
    DecodedSettings _decodeTxPacket(const TXPacket& packet);
    uint16_t _calculateSpeedRawFromThrottle(const TXPacket& packet);
    void _printParsedData();
    void _checkAndTriggerCallbacks();

    // --- Member Variables ---
    HardwareSerial* _serialPort;
    Stream* _debugPort;
    bool _debugEnabled = false;

    uint8_t _rxBuffer[sizeof(TXPacket)];
    uint8_t _rxIndex = 0;
    unsigned long _lastByteReceivedTime = 0;
    unsigned long _packetCounter = 0;

    TXPacket _txPacket;
    TXPacket _lastTxPacket;
    DecodedSettings _decodedSettings;

    bool _throttleSimulationEnabled = false; // Default to expecting external sensor data
    uint16_t _externalSpeedRaw = 0;

    // --- Callback Storage ---
    StateChangeCallback _onLightChange;
    StateChangeCallback _onBrakeChange;
    StateChangeCallback _onHornChange;
    StateChangeCallback _onBlinkerLeftChange;
    StateChangeCallback _onBlinkerRightChange;
    DriveModeChangeCallback _onDriveModeChange;
    ThrottleChangeCallback _onThrottleChange;
};

#endif // KUKIRING2PRO_H

