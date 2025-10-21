#ifndef KUKIRIN_G2_PRO_H
#define KUKIRIN_G2_PRO_H

#include <Arduino.h>
#include <functional>

// Define a generic callback function type
template<typename T>
using Callback = std::function<void(T)>;

class KukirinG2Pro {
public:
    // --- Public API ---

    // Constructor
    KukirinG2Pro();

    // Initialization
    void begin(HardwareSerial& displaySerial, Stream& debugSerial);
    void begin(HardwareSerial& displaySerial);

    // Main update function (must be called in loop)
    bool update();

    // Configuration
    void enableThrottleSimulation(bool enable);
    void enableDebugOutput(bool enable); 

    // Speed Control
    void setSpeedRaw(uint16_t speedRaw);

    // --- State Getters ---
    uint8_t getDriveMode() const;
    uint16_t getThrottle() const;
    bool isLightOn() const;
    bool isBrakeActive() const;
    bool isHornActive() const;
    bool isBlinkerLeftOn() const;
    bool isBlinkerRightOn() const;
    bool isSpeedProfileOpen() const;
    uint8_t getRecupLevel() const;
    uint8_t getAccelLevel() const;
    uint16_t getPoleCount() const;
    uint8_t getWheelInches() const;
    uint8_t getBatteryVolts() const;
    
    // --- Event Callbacks ---
    void onLightChange(Callback<bool> callback);
    void onBrakeChange(Callback<bool> callback);
    void onHornChange(Callback<bool> callback);
    void onBlinkerLeftChange(Callback<bool> callback);
    void onBlinkerRightChange(Callback<bool> callback);
    void onDriveModeChange(Callback<uint8_t> callback);
    void onThrottleChange(Callback<uint16_t> callback);
    void onSpeedProfileChange(Callback<bool> callback);
    void onRecupLevelChange(Callback<uint8_t> callback);
    void onAccelLevelChange(Callback<uint8_t> callback);

private:
    // --- Internal Structures ---
    struct __attribute__((packed)) RXPacket {
        uint8_t startMarker = 0x02, packetLength = 0x0E, statusType = 0x01;
        uint8_t statusFlag, systemStatus;
        uint8_t speedField[3] = {0,0,0};
        uint8_t speedRaw_H, speedRaw_L;
        uint8_t currentField[2] = {0,0};
        uint8_t calculatedStatus;
        uint8_t unknown_0x0D = 0x00;
        uint8_t echo_H = 0x02, echo_L = 0x0E;
    };

    struct __attribute__((packed)) TXPacket {
        uint8_t startMarker, packetLength, commandType, subCommand, driveMode, functionBitmask;
        uint8_t poleCount_L, poleCount_H, wheelCirc_L, wheelCirc_H, recupAccByte, reserved_0x0B;
        uint8_t speedProfile_L, speedProfile_H, batteryConfig_L, batteryConfig_H;
        uint8_t throttle_H, throttle_L, indicator_L, indicator_H;
    };

    struct DecodedSettings {
        uint8_t rekupLevel, accLevel, wheelInches, batteryVolts;
        uint16_t poleCount;
    };

    // --- Internal Methods ---
    void _resetReceiver();
    bool _receivePacket();
    bool _isPacketSane() const;
    void _parseDisplayPacket();
    void _sendResponsePacket();
    void _printParsedData();
    uint16_t _calculateSpeedRaw() const;
    uint8_t _calculateSpeedChecksum(uint8_t speed_H, uint8_t speed_L) const;
    DecodedSettings _decodeTxPacket() const;

    // --- Member Variables ---
    HardwareSerial* _displaySerial = nullptr;
    Stream* _debugSerial = nullptr;

    TXPacket _txPacket;
    RXPacket _rxPacket;
    uint8_t _rxBuffer[20];
    uint8_t _rxIndex = 0;
    unsigned long _lastByteReceivedTime = 0;
    unsigned long _packetCounter = 0;

    bool _simulationEnabled = false;
    bool _debugEnabled = false;
    uint16_t _externalSpeedRaw = 3500;

    // --- State tracking for callbacks ---
    uint8_t _lastDriveMode = 0;
    uint16_t _lastThrottle = 0;

    // --- Debouncing for Settings ---
    bool _lastConfirmedProfileIsOpen = false;
    bool _pendingProfileIsOpen = false;
    uint8_t _profileConfirmCounter = 0;

    uint8_t _lastConfirmedRecupLevel = 0;
    uint8_t _pendingRecupLevel = 0;
    uint8_t _recupConfirmCounter = 0;

    uint8_t _lastConfirmedAccelLevel = 0;
    uint8_t _pendingAccelLevel = 0;
    uint8_t _accelConfirmCounter = 0;
    
    bool _lastConfirmedLightState = false;
    bool _pendingLightState = false;
    uint8_t _lightConfirmCounter = 0;

    bool _lastConfirmedHornState = false;
    bool _pendingHornState = false;
    uint8_t _hornConfirmCounter = 0;

    bool _lastConfirmedBlinkerLeftState = false;
    bool _pendingBlinkerLeftState = false;
    uint8_t _blinkerLeftConfirmCounter = 0;

    bool _lastConfirmedBlinkerRightState = false;
    bool _pendingBlinkerRightState = false;
    uint8_t _blinkerRightConfirmCounter = 0;

    bool _lastConfirmedBrakeState = false;
    bool _pendingBrakeState = false;
    uint8_t _brakeConfirmCounter = 0;

    const uint8_t SETTING_CONFIRM_COUNT = 5; // Threshold for state change

    // --- Callback functions ---
    Callback<bool> _lightChangeCallback = nullptr;
    Callback<bool> _brakeChangeCallback = nullptr;
    Callback<bool> _hornChangeCallback = nullptr;
    Callback<bool> _blinkerLeftChangeCallback = nullptr;
    Callback<bool> _blinkerRightChangeCallback = nullptr;
    Callback<uint8_t> _driveModeChangeCallback = nullptr;
    Callback<uint16_t> _throttleChangeCallback = nullptr;
    Callback<bool> _speedProfileChangeCallback = nullptr;
    Callback<uint8_t> _recupLevelChangeCallback = nullptr;
    Callback<uint8_t> _accelLevelChangeCallback = nullptr;
};

#endif // KUKIRIN_G2_PRO_H


