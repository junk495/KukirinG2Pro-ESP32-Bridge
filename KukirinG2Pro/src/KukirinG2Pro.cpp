#include "KukirinG2Pro.h"

// --- Protocol Constants ---
const uint8_t PACKET_START_BYTE = 0x01;
const uint8_t PACKET_LENGTH_BYTE = 0x14;
const unsigned long RX_TIMEOUT_MS = 50;
const uint16_t PROFILE_LIMITED = 0x0C19;
const uint16_t PROFILE_OPEN = 0x0C64;
const uint8_t BRAKE_INDICATOR_VALUE = 0x25;
const uint8_t STATUS_HANDSHAKE_CALC = 0xEC;
const uint8_t STATUS_BRAKE_CALC = 0x4C;
const uint8_t SPEED_CHECKSUM_KEY = 0xCD;

// ============================================================================
// --- Public API Implementation ---
// ============================================================================

KukirinG2Pro::KukirinG2Pro() {}

void KukirinG2Pro::begin(HardwareSerial& displaySerial, Stream& debugSerial) {
    _displaySerial = &displaySerial;
    _debugSerial = &debugSerial;
    _displaySerial->begin(9600);
    _displaySerial->flush();
    delay(100);
}

void KukirinG2Pro::begin(HardwareSerial& displaySerial) {
    _displaySerial = &displaySerial;
    _debugSerial = nullptr;
    _displaySerial->begin(9600);
    _displaySerial->flush();
    delay(100);
}

bool KukirinG2Pro::update() {
    if (_receivePacket()) {
        _parseDisplayPacket();
        _sendResponsePacket();
        return true;
    }
    return false;
}

void KukirinG2Pro::enableThrottleSimulation(bool enable) {
    _simulationEnabled = enable;
}

void KukirinG2Pro::enableDebugOutput(bool enable) {
    _debugEnabled = enable;
}

void KukirinG2Pro::setSpeedRaw(uint16_t speedRaw) {
    _simulationEnabled = false;
    _externalSpeedRaw = speedRaw;
}

// --- Getters ---
uint8_t KukirinG2Pro::getDriveMode() const { return _txPacket.driveMode; }
uint16_t KukirinG2Pro::getThrottle() const { return (_txPacket.throttle_H << 8) | _txPacket.throttle_L; }
bool KukirinG2Pro::isLightOn() const { return _txPacket.functionBitmask & 0x20; }
bool KukirinG2Pro::isBrakeActive() const { return (_txPacket.indicator_L & 0x20) != 0; }
bool KukirinG2Pro::isHornActive() const { return _txPacket.indicator_L & 0x80; }
bool KukirinG2Pro::isBlinkerLeftOn() const { return _txPacket.indicator_L & 0x08; }
bool KukirinG2Pro::isBlinkerRightOn() const { return _txPacket.indicator_L & 0x10; }
bool KukirinG2Pro::isSpeedProfileOpen() const { 
    uint16_t profile = (_txPacket.speedProfile_H << 8) | _txPacket.speedProfile_L;
    return profile == PROFILE_OPEN;
}
uint8_t KukirinG2Pro::getRecupLevel() const { return _decodeTxPacket().rekupLevel; }
uint8_t KukirinG2Pro::getAccelLevel() const { return _decodeTxPacket().accLevel; }
uint16_t KukirinG2Pro::getPoleCount() const { return _decodeTxPacket().poleCount; }
uint8_t KukirinG2Pro::getWheelInches() const { return _decodeTxPacket().wheelInches; }
uint8_t KukirinG2Pro::getBatteryVolts() const { return _decodeTxPacket().batteryVolts; }

// --- Callbacks ---
void KukirinG2Pro::onLightChange(Callback<bool> callback) { _lightChangeCallback = callback; }
void KukirinG2Pro::onBrakeChange(Callback<bool> callback) { _brakeChangeCallback = callback; }
void KukirinG2Pro::onHornChange(Callback<bool> callback) { _hornChangeCallback = callback; }
void KukirinG2Pro::onBlinkerLeftChange(Callback<bool> callback) { _blinkerLeftChangeCallback = callback; }
void KukirinG2Pro::onBlinkerRightChange(Callback<bool> callback) { _blinkerRightChangeCallback = callback; }
void KukirinG2Pro::onDriveModeChange(Callback<uint8_t> callback) { _driveModeChangeCallback = callback; }
void KukirinG2Pro::onThrottleChange(Callback<uint16_t> callback) { _throttleChangeCallback = callback; }
void KukirinG2Pro::onSpeedProfileChange(Callback<bool> callback) { _speedProfileChangeCallback = callback; }
void KukirinG2Pro::onRecupLevelChange(Callback<uint8_t> callback) { _recupLevelChangeCallback = callback; }
void KukirinG2Pro::onAccelLevelChange(Callback<uint8_t> callback) { _accelLevelChangeCallback = callback; }

// ============================================================================
// --- Private Implementation ---
// ============================================================================

void KukirinG2Pro::_resetReceiver() {
    _rxIndex = 0;
    if (_displaySerial) {
        _displaySerial->flush();
    }
}

bool KukirinG2Pro::_isPacketSane() const {
    // Plausibilitätsprüfung für empfangene Daten
    uint16_t throttle = getThrottle();
    if (throttle > 1024) { // Max. Gaswert ist ~1000, 1024 als sichere Obergrenze
        if (_debugEnabled && _debugSerial) {
            _debugSerial->print("Packet discarded: Invalid throttle value -> ");
            _debugSerial->println(throttle);
        }
        return false;
    }

    uint8_t driveMode = getDriveMode();
    if (driveMode != 0x05 && driveMode != 0x0A && driveMode != 0x0F) {
        if (_debugEnabled && _debugSerial) {
            _debugSerial->print("Packet discarded: Invalid drive mode -> 0x");
            _debugSerial->println(driveMode, HEX);
        }
        return false;
    }
    return true; // Paket scheint gültig
}

bool KukirinG2Pro::_receivePacket() {
    while (_displaySerial->available()) {
        uint8_t inByte = _displaySerial->read();
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
                _resetReceiver();
            }
        } else {
            if (_rxIndex < sizeof(_rxBuffer)) {
                _rxBuffer[_rxIndex++] = inByte;
                _lastByteReceivedTime = millis();
            }
        }
        
        if (_rxIndex >= sizeof(TXPacket)) {
            memcpy(&_txPacket, _rxBuffer, sizeof(TXPacket));
            
            if (!_isPacketSane()) {
                _resetReceiver();
                return false; 
            }
            
            _resetReceiver();
            return true; 
        }
    }

    if (_rxIndex > 0 && (millis() - _lastByteReceivedTime > RX_TIMEOUT_MS)) {
        if (_debugEnabled && _debugSerial) _debugSerial->println("RX Timeout! Resyncing...");
        _resetReceiver();
    }
    return false;
}

void KukirinG2Pro::_parseDisplayPacket() {
    _packetCounter++;

    // --- Trigger Callbacks on Change (non-debounced for critical inputs) ---
    uint8_t currentDriveMode = getDriveMode();
    if(currentDriveMode != _lastDriveMode) {
        if(_driveModeChangeCallback) _driveModeChangeCallback(currentDriveMode);
        _lastDriveMode = currentDriveMode;
    }
    uint16_t currentThrottle = getThrottle();
    if(currentThrottle != _lastThrottle) {
        if(_throttleChangeCallback) _throttleChangeCallback(currentThrottle);
        _lastThrottle = currentThrottle;
    }

    // --- Debounce Brake State ---
    bool currentBrakeState = isBrakeActive();
    if (currentBrakeState == _pendingBrakeState) {
        if (_brakeConfirmCounter < SETTING_CONFIRM_COUNT) {
            _brakeConfirmCounter++;
        }
    } else {
        _pendingBrakeState = currentBrakeState;
        _brakeConfirmCounter = 1;
    }

    if (_brakeConfirmCounter >= SETTING_CONFIRM_COUNT && _pendingBrakeState != _lastConfirmedBrakeState) {
        _lastConfirmedBrakeState = _pendingBrakeState;
        if (_brakeChangeCallback) {
            _brakeChangeCallback(_lastConfirmedBrakeState);
        }
    }

    // --- Debounce Light State ---
    bool currentLightState = isLightOn();
    if (currentLightState == _pendingLightState) {
        if (_lightConfirmCounter < SETTING_CONFIRM_COUNT) {
            _lightConfirmCounter++;
        }
    } else {
        _pendingLightState = currentLightState;
        _lightConfirmCounter = 1;
    }

    if (_lightConfirmCounter >= SETTING_CONFIRM_COUNT && _pendingLightState != _lastConfirmedLightState) {
        _lastConfirmedLightState = _pendingLightState;
        if (_lightChangeCallback) {
            _lightChangeCallback(_lastConfirmedLightState);
        }
    }

    // --- Debounce Horn State ---
    bool currentHornState = isHornActive();
    if (currentHornState == _pendingHornState) {
        if (_hornConfirmCounter < SETTING_CONFIRM_COUNT) {
            _hornConfirmCounter++;
        }
    } else {
        _pendingHornState = currentHornState;
        _hornConfirmCounter = 1;
    }

    if (_hornConfirmCounter >= SETTING_CONFIRM_COUNT && _pendingHornState != _lastConfirmedHornState) {
        _lastConfirmedHornState = _pendingHornState;
        if (_hornChangeCallback) {
            _hornChangeCallback(_lastConfirmedHornState);
        }
    }

    // --- Debounce Blinker Left State ---
    bool currentBlinkerLeftState = isBlinkerLeftOn();
    if (currentBlinkerLeftState == _pendingBlinkerLeftState) {
        if (_blinkerLeftConfirmCounter < SETTING_CONFIRM_COUNT) {
            _blinkerLeftConfirmCounter++;
        }
    } else {
        _pendingBlinkerLeftState = currentBlinkerLeftState;
        _blinkerLeftConfirmCounter = 1;
    }

    if (_blinkerLeftConfirmCounter >= SETTING_CONFIRM_COUNT && _pendingBlinkerLeftState != _lastConfirmedBlinkerLeftState) {
        _lastConfirmedBlinkerLeftState = _pendingBlinkerLeftState;
        if (_blinkerLeftChangeCallback) {
            _blinkerLeftChangeCallback(_lastConfirmedBlinkerLeftState);
        }
    }

    // --- Debounce Blinker Right State ---
    bool currentBlinkerRightState = isBlinkerRightOn();
    if (currentBlinkerRightState == _pendingBlinkerRightState) {
        if (_blinkerRightConfirmCounter < SETTING_CONFIRM_COUNT) {
            _blinkerRightConfirmCounter++;
        }
    } else {
        _pendingBlinkerRightState = currentBlinkerRightState;
        _blinkerRightConfirmCounter = 1;
    }

    if (_blinkerRightConfirmCounter >= SETTING_CONFIRM_COUNT && _pendingBlinkerRightState != _lastConfirmedBlinkerRightState) {
        _lastConfirmedBlinkerRightState = _pendingBlinkerRightState;
        if (_blinkerRightChangeCallback) {
            _blinkerRightChangeCallback(_lastConfirmedBlinkerRightState);
        }
    }

    // --- Debounce Speed Profile State ---
    bool currentProfileIsOpen = isSpeedProfileOpen();

    if (currentProfileIsOpen == _pendingProfileIsOpen) {
        if (_profileConfirmCounter < SETTING_CONFIRM_COUNT) {
            _profileConfirmCounter++;
        }
    } else {
        _pendingProfileIsOpen = currentProfileIsOpen;
        _profileConfirmCounter = 1;
    }

    if (_profileConfirmCounter >= SETTING_CONFIRM_COUNT && _pendingProfileIsOpen != _lastConfirmedProfileIsOpen) {
        _lastConfirmedProfileIsOpen = _pendingProfileIsOpen;
        if (_speedProfileChangeCallback) {
            _speedProfileChangeCallback(_lastConfirmedProfileIsOpen);
        }
    }

    // --- Debounce Recuperation Level (PB) ---
    uint8_t currentRecupLevel = getRecupLevel();
    if (currentRecupLevel == _pendingRecupLevel) {
        if (_recupConfirmCounter < SETTING_CONFIRM_COUNT) {
            _recupConfirmCounter++;
        }
    } else {
        _pendingRecupLevel = currentRecupLevel;
        _recupConfirmCounter = 1;
    }

    if (_recupConfirmCounter >= SETTING_CONFIRM_COUNT && _pendingRecupLevel != _lastConfirmedRecupLevel) {
        _lastConfirmedRecupLevel = _pendingRecupLevel;
        if (_recupLevelChangeCallback) {
            _recupLevelChangeCallback(_lastConfirmedRecupLevel);
        }
    }

    // --- Debounce Acceleration Level (PA) ---
    uint8_t currentAccelLevel = getAccelLevel();
    if (currentAccelLevel == _pendingAccelLevel) {
        if (_accelConfirmCounter < SETTING_CONFIRM_COUNT) {
            _accelConfirmCounter++;
        }
    } else {
        _pendingAccelLevel = currentAccelLevel;
        _accelConfirmCounter = 1;
    }

    if (_accelConfirmCounter >= SETTING_CONFIRM_COUNT && _pendingAccelLevel != _lastConfirmedAccelLevel) {
        _lastConfirmedAccelLevel = _pendingAccelLevel;
        if (_accelLevelChangeCallback) {
            _accelLevelChangeCallback(_lastConfirmedAccelLevel);
        }
    }

    _printParsedData();
}

void KukirinG2Pro::_sendResponsePacket() {
    uint16_t speedRaw = _simulationEnabled ? _calculateSpeedRaw() : _externalSpeedRaw;
    _rxPacket.speedRaw_H = (speedRaw >> 8) & 0xFF;
    _rxPacket.speedRaw_L = speedRaw & 0xFF;

    bool brake = isBrakeActive();
    if (_packetCounter % 50 == 2) {
        _rxPacket.statusFlag = 0x80;
        _rxPacket.systemStatus = 0xC0;
        _rxPacket.calculatedStatus = STATUS_HANDSHAKE_CALC;
    } else if (brake) {
        _rxPacket.statusFlag = 0x00;
        _rxPacket.systemStatus = 0xE0;
        _rxPacket.calculatedStatus = STATUS_BRAKE_CALC;
    } else {
        _rxPacket.statusFlag = 0x00;
        _rxPacket.systemStatus = 0xC0;
        _rxPacket.calculatedStatus = _calculateSpeedChecksum(_rxPacket.speedRaw_H, _rxPacket.speedRaw_L);
    }
    
    _displaySerial->write((uint8_t*)&_rxPacket, sizeof(RXPacket));
}

void KukirinG2Pro::_printParsedData() {
    if (!_debugEnabled || !_debugSerial) return;
    
    _debugSerial->println("========== Display Commands ==========");
    _debugSerial->print("Mode: L");
    switch (getDriveMode()) {
        case 0x05: _debugSerial->print("1"); break;
        case 0x0A: _debugSerial->print("2"); break;
        case 0x0F: _debugSerial->print("3"); break;
        default: _debugSerial->print("?"); break;
    }
    _debugSerial->print(" | Profile: ");
    _debugSerial->print(isSpeedProfileOpen() ? "Open" : "Limited");
    _debugSerial->print(" | Throttle: ");
    _debugSerial->println(getThrottle());

    DecodedSettings s = _decodeTxPacket();
    _debugSerial->print("Recup: L");
    _debugSerial->print(s.rekupLevel);
    _debugSerial->print(" (PB) | Accel: L");
    _debugSerial->print(s.accLevel);
    _debugSerial->println(" (PA)");

    _debugSerial->print("Indicators: ");
    bool hasIndicator = false;
    if (isBrakeActive()) { _debugSerial->print("BRAKE "); hasIndicator = true; }
    if (isLightOn()) { _debugSerial->print("LIGHT "); hasIndicator = true; }
    if (isBlinkerLeftOn()) { _debugSerial->print("BLINK_L "); hasIndicator = true; }
    if (isBlinkerRightOn()) { _debugSerial->print("BLINK_R "); hasIndicator = true; }
    if (isHornActive()) { _debugSerial->print("HORN "); hasIndicator = true; }
    if (!hasIndicator) { _debugSerial->print("None"); }
    _debugSerial->println("\n");
}

KukirinG2Pro::DecodedSettings KukirinG2Pro::_decodeTxPacket() const {
    DecodedSettings s;
    s.rekupLevel = (_txPacket.recupAccByte >> 4) & 0x0F;
    s.accLevel = _txPacket.recupAccByte & 0x0F;
    s.poleCount = (_txPacket.poleCount_H << 8) | _txPacket.poleCount_L;
    uint16_t wheel_cfg = (_txPacket.wheelCirc_H << 8) | _txPacket.wheelCirc_L;
    s.wheelInches = (wheel_cfg > 658) ? (wheel_cfg - 658) / 2 : 0;
    uint16_t batt_cfg = (_txPacket.batteryConfig_H << 8) | _txPacket.batteryConfig_L;
    s.batteryVolts = (batt_cfg > 60) ? (batt_cfg + 60) / 10 : 0;
    return s;
}

uint16_t KukirinG2Pro::_calculateSpeedRaw() const {
    uint16_t throttle = getThrottle();
    if (throttle < 14) return 3500;

    float vMax = 25.0;
    uint16_t maxThrottle = 1000;
    
    if (isSpeedProfileOpen()) {
        switch (getDriveMode()) {
            case 0x05: vMax = 19.0; maxThrottle = 360; break;
            case 0x0A: vMax = 38.0; maxThrottle = 680; break;
            case 0x0F: vMax = 53.0; maxThrottle = 1000; break;
        }
    } else { // Limited Profile
        switch (getDriveMode()) {
            case 0x05: vMax = 15.0; maxThrottle = 360; break;
            case 0x0A: vMax = 20.0; maxThrottle = 680; break;
            case 0x0F: vMax = 25.0; maxThrottle = 1000; break;
        }
    }
    
    if (throttle > maxThrottle) throttle = maxThrottle;
    
    float throttlePercent = (float)throttle / (float)maxThrottle;
    float currentSpeed = throttlePercent * vMax;
    
    if (currentSpeed < 0.5) return 3500;

    DecodedSettings s = _decodeTxPacket();
    uint16_t poleCount = s.poleCount > 0 ? s.poleCount : 30;
    uint8_t wheelInches = s.wheelInches > 0 ? s.wheelInches : 100;

    float pole_adj = 30.0 / poleCount;
    float wheel_adj = 100.0 / wheelInches;
    float adj_const = 2550.0 * pole_adj * wheel_adj;

    uint16_t speedRaw = (uint16_t)(adj_const / currentSpeed);
    
    if (speedRaw < 0x0030) speedRaw = 0x0030;
    if (speedRaw > 3500) speedRaw = 3500;
    
    return speedRaw;
}

uint8_t KukirinG2Pro::_calculateSpeedChecksum(uint8_t speed_H, uint8_t speed_L) const {
    return (speed_H ^ speed_L) ^ SPEED_CHECKSUM_KEY;
}



