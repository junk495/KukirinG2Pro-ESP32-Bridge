#include "Arduino.h"
#include "KukirinG2Pro.h"

// RX Packet (Controller -> Display, 16 Bytes)
struct __attribute__((packed)) RXPacket {
    uint8_t  startMarker;
    uint8_t  packetLength;
    uint8_t  statusType;
    uint8_t  statusFlag;
    uint8_t  systemStatus;
    uint8_t  speedField[3];
    uint8_t  speedRaw_H;
    uint8_t  speedRaw_L;
    uint8_t  currentField[2];
    uint8_t  calculatedStatus;
    uint8_t  unknown_0x0D;
    uint8_t  echo_H;
    uint8_t  echo_L;
};

// --- Constants ---
const uint16_t HANDSHAKE_INTERVAL = 50;
const unsigned long RX_TIMEOUT_MS = 50;
const uint16_t SPEED_RAW_IDLE = 0x0DAC;
const uint8_t BRAKE_INDICATOR_VALUE = 0x25;
const uint8_t PACKET_START_BYTE = 0x01;
const uint8_t PACKET_LENGTH_BYTE = 0x14;
const uint8_t STATUS_HANDSHAKE_CALC = 0xEC;
const uint8_t STATUS_BRAKE_CALC = 0x4C;
const uint8_t SPEED_CHECKSUM_KEY = 0xCD;

KukirinG2Pro::KukirinG2Pro() {
    _externalSpeedRaw = SPEED_RAW_IDLE; // Start with idle speed
}

void KukirinG2Pro::begin(HardwareSerial& port, Stream& debugPort) {
    begin(port);
    _debugPort = &debugPort;
    _debugEnabled = true;
}

void KukirinG2Pro::begin(HardwareSerial& port) {
    _serialPort = &port;
    _serialPort->begin(9600);
    _serialPort->flush();
    delay(100);
    memset(&_lastTxPacket, 0, sizeof(TXPacket));
    _lastByteReceivedTime = millis();
}

bool KukirinG2Pro::update() {
    bool newPacketReceived = false;
    while (_serialPort->available()) {
        uint8_t inByte = _serialPort->read();
        if (_rxIndex == 0) {
            if (inByte == PACKET_START_BYTE) {
                _rxBuffer[_rxIndex++] = inByte;
                _lastByteReceivedTime = millis();
            }
        } else if (_rxIndex == 1) {
            if (inByte == PACKET_LENGTH_BYTE) {
                _rxBuffer[_rxIndex++] = inByte;
                _lastByteReceivedTime = millis();
            } else {
                _rxIndex = 0;
            }
        } else {
            if (_rxIndex < sizeof(_rxBuffer)) {
                _rxBuffer[_rxIndex++] = inByte;
                _lastByteReceivedTime = millis();
            }
        }
        
        if (_rxIndex >= sizeof(TXPacket)) {
            _handleDisplayPacket();
            _rxIndex = 0;
            newPacketReceived = true;
        }
    }

    if (_rxIndex > 0 && (millis() - _lastByteReceivedTime > RX_TIMEOUT_MS)) {
        if(_debugEnabled) {
            _debugPort->print("RX Timeout! Discarding ");
            _debugPort->print(_rxIndex);
            _debugPort->println(" bytes. Resyncing...");
        }
        _rxIndex = 0;
        _serialPort->flush();
    }
    return newPacketReceived;
}

void KukirinG2Pro::setSpeedRaw(uint16_t speedRaw) {
    // Receiving an external speed value automatically disables simulation
    if (_throttleSimulationEnabled) {
        _throttleSimulationEnabled = false;
    }
    _externalSpeedRaw = speedRaw;
}

void KukirinG2Pro::enableThrottleSimulation(bool enable) {
    _throttleSimulationEnabled = enable;
}

void KukirinG2Pro::_handleDisplayPacket() {
    memcpy(&_txPacket, _rxBuffer, sizeof(TXPacket));
    
    // Check for changes and trigger callbacks before updating lastTxPacket
    _checkAndTriggerCallbacks();
    
    // Update settings and print debug info if anything changed
    if (memcmp(&_txPacket, &_lastTxPacket, sizeof(TXPacket)) != 0) {
        _decodedSettings = _decodeTxPacket(_txPacket);
        if (_debugEnabled) {
            _printParsedData();
        }
    }
    
    memcpy(&_lastTxPacket, &_txPacket, sizeof(TXPacket));
    _packetCounter++;
    _sendResponse();
}

void KukirinG2Pro::_sendResponse() {
    RXPacket rxPacket;
    rxPacket.startMarker = 0x02;
    rxPacket.packetLength = 0x0E;
    rxPacket.statusType = 0x01;
    memset(rxPacket.speedField, 0, sizeof(rxPacket.speedField));
    memset(rxPacket.currentField, 0, sizeof(rxPacket.currentField));
    rxPacket.unknown_0x0D = 0x00;
    rxPacket.echo_H = 0x02;
    rxPacket.echo_L = 0x0E;

    uint16_t speedRaw = _throttleSimulationEnabled ? _calculateSpeedRawFromThrottle(_txPacket) : _externalSpeedRaw;
    
    rxPacket.speedRaw_H = (speedRaw >> 8) & 0xFF;
    rxPacket.speedRaw_L = speedRaw & 0xFF;

    bool currentBrakeState = (_txPacket.indicator_L == BRAKE_INDICATOR_VALUE);
    uint16_t cyclePosition = _packetCounter % HANDSHAKE_INTERVAL;

    if (cyclePosition == 2) {
        rxPacket.statusFlag = 0x80;
        rxPacket.systemStatus = 0xC0;
        rxPacket.calculatedStatus = STATUS_HANDSHAKE_CALC;
    } else if (currentBrakeState) {
        rxPacket.statusFlag = 0x00;
        rxPacket.systemStatus = 0xE0;
        rxPacket.calculatedStatus = STATUS_BRAKE_CALC;
    } else {
        rxPacket.statusFlag = 0x00;
        rxPacket.systemStatus = 0xC0;
        rxPacket.calculatedStatus = _calculateSpeedChecksum(rxPacket.speedRaw_H, rxPacket.speedRaw_L);
    }
    
    _serialPort->write((uint8_t*)&rxPacket, sizeof(RXPacket));
}

// --- Getter Methods ---
uint16_t KukirinG2Pro::getThrottle() { return (_txPacket.throttle_H << 8) | _txPacket.throttle_L; }
bool KukirinG2Pro::isBrakeActive() { return _txPacket.indicator_L == BRAKE_INDICATOR_VALUE; }
bool KukirinG2Pro::isLightOn() { return _txPacket.functionBitmask & 0x20; }
bool KukirinG2Pro::isHornOn() { return _txPacket.indicator_L & 0x80; }
bool KukirinG2Pro::isBlinkerLeftOn() { return _txPacket.indicator_L & 0x08; }
bool KukirinG2Pro::isBlinkerRightOn() { return _txPacket.indicator_L & 0x10; }
const DecodedSettings& KukirinG2Pro::getSettings() { return _decodedSettings; }
const TXPacket& KukirinG2Pro::getRawTXPacket() { return _txPacket; }

// --- Callback Registration ---
void KukirinG2Pro::onLightChange(StateChangeCallback callback) { _onLightChange = callback; }
void KukirinG2Pro::onBrakeChange(StateChangeCallback callback) { _onBrakeChange = callback; }
void KukirinG2Pro::onHornChange(StateChangeCallback callback) { _onHornChange = callback; }
void KukirinG2Pro::onBlinkerLeftChange(StateChangeCallback callback) { _onBlinkerLeftChange = callback; }
void KukirinG2Pro::onBlinkerRightChange(StateChangeCallback callback) { _onBlinkerRightChange = callback; }
void KukirinG2Pro::onDriveModeChange(DriveModeChangeCallback callback) { _onDriveModeChange = callback; }
void KukirinG2Pro::onThrottleChange(ThrottleChangeCallback callback) { _onThrottleChange = callback; }


// --- Private Helper Functions ---
void KukirinG2Pro::_checkAndTriggerCallbacks() {
    // Check for light state change
    if (_onLightChange && (isLightOn() != ((_lastTxPacket.functionBitmask & 0x20) > 0))) {
        _onLightChange(isLightOn());
    }
    // Check for brake state change
    if (_onBrakeChange && (isBrakeActive() != (_lastTxPacket.indicator_L == BRAKE_INDICATOR_VALUE))) {
        _onBrakeChange(isBrakeActive());
    }
    // Check for horn state change
    if (_onHornChange && (isHornOn() != ((_lastTxPacket.indicator_L & 0x80) > 0))) {
        _onHornChange(isHornOn());
    }
    // Check for blinker left state change
    if (_onBlinkerLeftChange && (isBlinkerLeftOn() != ((_lastTxPacket.indicator_L & 0x08) > 0))) {
        _onBlinkerLeftChange(isBlinkerLeftOn());
    }
    // Check for blinker right state change
    if (_onBlinkerRightChange && (isBlinkerRightOn() != ((_lastTxPacket.indicator_L & 0x10) > 0))) {
        _onBlinkerRightChange(isBlinkerRightOn());
    }
    // Check for drive mode change
    if (_onDriveModeChange && (_txPacket.driveMode != _lastTxPacket.driveMode)) {
        _onDriveModeChange(_txPacket.driveMode);
    }
    // Check for throttle change
    if (_onThrottleChange && (getThrottle() != ((_lastTxPacket.throttle_H << 8) | _lastTxPacket.throttle_L))) {
        _onThrottleChange(getThrottle());
    }
}

uint8_t KukirinG2Pro::_calculateSpeedChecksum(uint8_t speed_H, uint8_t speed_L) {
    return (speed_H ^ speed_L) ^ SPEED_CHECKSUM_KEY;
}

DecodedSettings KukirinG2Pro::_decodeTxPacket(const TXPacket& packet) {
    DecodedSettings settings;
    settings.rekupLevel = (packet.recupAccByte >> 4) & 0x0F;
    settings.accLevel = packet.recupAccByte & 0x0F;
    settings.poleCount = packet.poleCount_L | (packet.poleCount_H << 8);
    uint16_t wheel_config = packet.wheelCirc_L | (packet.wheelCirc_H << 8);
    settings.wheelInches = (wheel_config > 658) ? (wheel_config - 658) / 2 : 0;
    uint16_t battery_config = packet.batteryConfig_L | (packet.batteryConfig_H << 8);
    settings.batteryVolts = (battery_config > 60) ? (battery_config + 60) / 10 : 0;
    return settings;
}

uint16_t KukirinG2Pro::_calculateSpeedRawFromThrottle(const TXPacket& packet) {
    uint16_t throttle = (_txPacket.throttle_H << 8) | _txPacket.throttle_L;
    if (throttle < 14) return SPEED_RAW_IDLE;
    
    float vMax = 25.0;
    uint16_t maxThrottle = 1000;
    uint16_t speedProfile = packet.speedProfile_L | (packet.speedProfile_H << 8);
    bool isLimited = (speedProfile == 0x0C19);
    
    if (isLimited) {
        if(packet.driveMode == 0x05) { vMax = 15.0; maxThrottle = 360; }
        else if(packet.driveMode == 0x0A) { vMax = 20.0; maxThrottle = 680; }
        else { vMax = 25.0; maxThrottle = 1000; }
    } else {
        if(packet.driveMode == 0x05) { vMax = 19.0; maxThrottle = 360; }
        else if(packet.driveMode == 0x0A) { vMax = 38.0; maxThrottle = 680; }
        else { vMax = 53.0; maxThrottle = 1000; }
    }
    
    if (throttle > maxThrottle) throttle = maxThrottle;
    
    float throttlePercent = (float)throttle / (float)maxThrottle;
    float currentSpeed = throttlePercent * vMax;
    
    if (currentSpeed < 0.5) return SPEED_RAW_IDLE;

    DecodedSettings settings = _decodeTxPacket(packet);
    uint16_t poleCount = settings.poleCount > 0 ? settings.poleCount : 30;
    uint8_t wheelInches = settings.wheelInches > 0 ? settings.wheelInches : 100;

    float pole_adjustment = 30.0 / poleCount;
    float wheel_adjustment = 100.0 / wheelInches;
    float adjusted_constant = 2550.0 * pole_adjustment * wheel_adjustment;

    uint16_t speedRaw = (uint16_t)(adjusted_constant / currentSpeed);
    
    if (speedRaw > SPEED_RAW_IDLE) speedRaw = SPEED_RAW_IDLE;
    if (speedRaw < 0x0030) speedRaw = 0x0030;
    
    return speedRaw;
}

void KukirinG2Pro::_printParsedData() {
    _debugPort->println("\n========== Display Commands ==========");
    _debugPort->print("Mode: L");
    switch (_txPacket.driveMode) {
        case 0x05: _debugPort->print("1"); break;
        case 0x0A: _debugPort->print("2"); break;
        case 0x0F: _debugPort->print("3"); break;
        default: _debugPort->print("?"); break;
    }

    _debugPort->print(" | Profile: ");
    uint16_t speedProfile = (_txPacket.speedProfile_L | (_txPacket.speedProfile_H << 8));
    if (speedProfile == 0x0C19) {
        _debugPort->print("Limited");
    } else if (speedProfile == 0x0C64) {
        _debugPort->print("Open");
    } else {
        _debugPort->print("Unknown");
    }

    _debugPort->print(" | Throttle: ");
    _debugPort->println(getThrottle());

    _debugPort->print("Recup: L");
    _debugPort->print(_decodedSettings.rekupLevel);
    _debugPort->print(" (PB) | Accel: L");
    _debugPort->print(_decodedSettings.accLevel);
    _debugPort->println(" (PA)");

    _debugPort->print("Indicators: ");
    bool hasIndicator = false;
    if(isBrakeActive()) { _debugPort->print("BRAKE "); hasIndicator = true; }
    if(isLightOn()) { _debugPort->print("LIGHT "); hasIndicator = true; }
    if(isHornOn()) { _debugPort->print("HORN "); hasIndicator = true; }
    if(isBlinkerLeftOn()) { _debugPort->print("BLINK_L "); hasIndicator = true; }
    if(isBlinkerRightOn()) { _debugPort->print("BLINK_R "); hasIndicator = true; }

    if(!hasIndicator) {
        _debugPort->print("None");
    }
    
    _debugPort->println();
}

