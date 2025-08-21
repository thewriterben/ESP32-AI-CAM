/*
  Wildlife Trail Camera Servo Control Implementation
  
  Provides pan/tilt functionality for wildlife tracking using SG90 micro servos
*/

#include "wildlife_servo_control.h"
#include "tcamera_plus_s3_config.h"

// Global servo control instance
WildlifeServoControl servoControl;

WildlifeServoControl::WildlifeServoControl() {
    currentPanAngle = 0;
    currentTiltAngle = 0;
    servosInitialized = false;
    servoPowerEnabled = false;
}

WildlifeServoControl::~WildlifeServoControl() {
    deinitializeServos();
}

bool WildlifeServoControl::initializeServos() {
    if (servosInitialized) {
        return true;
    }
    
    Serial.println("Initializing wildlife camera servos...");
    
    // Enable servo power
    enableServoPower();
    delay(100);
    
    // Configure servo pins and attach servos
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    
    panServo.setPeriodHertz(SERVO_FREQUENCY);
    tiltServo.setPeriodHertz(SERVO_FREQUENCY);
    
    if (!panServo.attach(PAN_SERVO_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE)) {
        Serial.println("Failed to attach pan servo");
        return false;
    }
    
    if (!tiltServo.attach(TILT_SERVO_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE)) {
        Serial.println("Failed to attach tilt servo");
        panServo.detach();
        return false;
    }
    
    // Move to home position
    moveToHomePosition();
    delay(1000); // Allow servos to reach position
    
    servosInitialized = true;
    Serial.println("Wildlife camera servos initialized successfully");
    
    return true;
}

void WildlifeServoControl::deinitializeServos() {
    if (!servosInitialized) {
        return;
    }
    
    Serial.println("Deinitializing servos...");
    
    // Move to rest position before detaching
    moveToRestPosition();
    delay(500);
    
    panServo.detach();
    tiltServo.detach();
    
    disableServoPower();
    
    servosInitialized = false;
    Serial.println("Servos deinitialized");
}

bool WildlifeServoControl::setPanAngle(int angle) {
    if (!servosInitialized) {
        Serial.println("Servos not initialized");
        return false;
    }
    
    int constrainedAngle = constrainPanAngle(angle);
    
    // Convert angle to servo position (0-180 range)
    int servoPosition = map(constrainedAngle, PAN_MIN_ANGLE, PAN_MAX_ANGLE, 0, 180);
    
    panServo.write(servoPosition);
    currentPanAngle = constrainedAngle;
    
    return true;
}

bool WildlifeServoControl::setTiltAngle(int angle) {
    if (!servosInitialized) {
        Serial.println("Servos not initialized");
        return false;
    }
    
    int constrainedAngle = constrainTiltAngle(angle);
    
    // Convert angle to servo position (0-180 range)
    int servoPosition = map(constrainedAngle, TILT_MIN_ANGLE, TILT_MAX_ANGLE, 0, 180);
    
    tiltServo.write(servoPosition);
    currentTiltAngle = constrainedAngle;
    
    return true;
}

bool WildlifeServoControl::setPosition(int panAngle, int tiltAngle) {
    bool panResult = setPanAngle(panAngle);
    bool tiltResult = setTiltAngle(tiltAngle);
    
    return panResult && tiltResult;
}

bool WildlifeServoControl::smoothPanTo(int targetAngle) {
    if (!servosInitialized) {
        return false;
    }
    
    int constrainedTarget = constrainPanAngle(targetAngle);
    int direction = (constrainedTarget > currentPanAngle) ? 1 : -1;
    
    while (currentPanAngle != constrainedTarget) {
        int nextAngle = currentPanAngle + (direction * STEP_SIZE);
        
        if ((direction > 0 && nextAngle > constrainedTarget) || 
            (direction < 0 && nextAngle < constrainedTarget)) {
            nextAngle = constrainedTarget;
        }
        
        setPanAngle(nextAngle);
        delayWithStatusUpdate(MOVEMENT_DELAY);
    }
    
    return true;
}

bool WildlifeServoControl::smoothTiltTo(int targetAngle) {
    if (!servosInitialized) {
        return false;
    }
    
    int constrainedTarget = constrainTiltAngle(targetAngle);
    int direction = (constrainedTarget > currentTiltAngle) ? 1 : -1;
    
    while (currentTiltAngle != constrainedTarget) {
        int nextAngle = currentTiltAngle + (direction * STEP_SIZE);
        
        if ((direction > 0 && nextAngle > constrainedTarget) || 
            (direction < 0 && nextAngle < constrainedTarget)) {
            nextAngle = constrainedTarget;
        }
        
        setTiltAngle(nextAngle);
        delayWithStatusUpdate(MOVEMENT_DELAY);
    }
    
    return true;
}

bool WildlifeServoControl::smoothMoveTo(int panAngle, int tiltAngle) {
    // Move both servos simultaneously for diagonal movements
    int targetPan = constrainPanAngle(panAngle);
    int targetTilt = constrainTiltAngle(tiltAngle);
    
    while (currentPanAngle != targetPan || currentTiltAngle != targetTilt) {
        bool panDone = (currentPanAngle == targetPan);
        bool tiltDone = (currentTiltAngle == targetTilt);
        
        if (!panDone) {
            int panDirection = (targetPan > currentPanAngle) ? 1 : -1;
            int nextPan = currentPanAngle + (panDirection * STEP_SIZE);
            if ((panDirection > 0 && nextPan > targetPan) || 
                (panDirection < 0 && nextPan < targetPan)) {
                nextPan = targetPan;
            }
            setPanAngle(nextPan);
        }
        
        if (!tiltDone) {
            int tiltDirection = (targetTilt > currentTiltAngle) ? 1 : -1;
            int nextTilt = currentTiltAngle + (tiltDirection * STEP_SIZE);
            if ((tiltDirection > 0 && nextTilt > targetTilt) || 
                (tiltDirection < 0 && nextTilt < targetTilt)) {
                nextTilt = targetTilt;
            }
            setTiltAngle(nextTilt);
        }
        
        delayWithStatusUpdate(MOVEMENT_DELAY);
    }
    
    return true;
}

bool WildlifeServoControl::moveToHomePosition() {
    Serial.println("Moving to home position (0°, 0°)");
    return setPosition(0, 0);
}

bool WildlifeServoControl::moveToScanPosition() {
    Serial.println("Moving to scan position (0°, 15°)");
    return smoothMoveTo(0, 15); // Slightly elevated for better wildlife detection
}

bool WildlifeServoControl::moveToRestPosition() {
    Serial.println("Moving to rest position (0°, -45°)");
    return smoothMoveTo(0, -45); // Downward angle to protect lens
}

bool WildlifeServoControl::performHorizontalScan(int startAngle, int endAngle) {
    Serial.printf("Performing horizontal scan from %d° to %d°\n", startAngle, endAngle);
    
    if (!smoothPanTo(startAngle)) {
        return false;
    }
    
    delay(500); // Pause at start position
    
    return smoothPanTo(endAngle);
}

bool WildlifeServoControl::performVerticalScan(int startAngle, int endAngle) {
    Serial.printf("Performing vertical scan from %d° to %d°\n", startAngle, endAngle);
    
    if (!smoothTiltTo(startAngle)) {
        return false;
    }
    
    delay(500); // Pause at start position
    
    return smoothTiltTo(endAngle);
}

bool WildlifeServoControl::performGridScan() {
    Serial.println("Performing grid scan pattern");
    
    // Grid scan pattern optimized for wildlife detection
    int panPositions[] = {-90, -45, 0, 45, 90};
    int tiltPositions[] = {-30, -15, 0, 15, 30};
    
    for (int t = 0; t < 5; t++) {
        for (int p = 0; p < 5; p++) {
            if (!smoothMoveTo(panPositions[p], tiltPositions[t])) {
                return false;
            }
            delay(1000); // Pause for motion detection
        }
    }
    
    return moveToHomePosition();
}

bool WildlifeServoControl::performWildlifeScan() {
    Serial.println("Performing wildlife-optimized scan");
    
    // Wildlife scan focuses on ground level and common animal paths
    int positions[][2] = {
        {-90, -15}, {-45, -10}, {0, 0}, {45, -10}, {90, -15},  // Ground level sweep
        {-60, 10}, {-30, 15}, {0, 20}, {30, 15}, {60, 10},     // Elevated sweep
        {0, 30}                                                  // Tree/sky check
    };
    
    for (int i = 0; i < 11; i++) {
        if (!smoothMoveTo(positions[i][0], positions[i][1])) {
            return false;
        }
        delay(2000); // Longer pause for wildlife detection
    }
    
    return moveToHomePosition();
}

bool WildlifeServoControl::trackTarget(int panAngle, int tiltAngle, bool smooth) {
    Serial.printf("Tracking target at pan: %d°, tilt: %d°\n", panAngle, tiltAngle);
    
    if (smooth) {
        return smoothMoveTo(panAngle, tiltAngle);
    } else {
        return setPosition(panAngle, tiltAngle);
    }
}

bool WildlifeServoControl::sweepArea(int centerPan, int centerTilt, int sweepRadius) {
    Serial.printf("Sweeping area around pan: %d°, tilt: %d°, radius: %d°\n", 
                  centerPan, centerTilt, sweepRadius);
    
    // Sweep in a small area around the detected motion
    for (int angle = -sweepRadius; angle <= sweepRadius; angle += 10) {
        if (!smoothMoveTo(centerPan + angle, centerTilt)) {
            return false;
        }
        delay(500);
    }
    
    // Return to center
    return smoothMoveTo(centerPan, centerTilt);
}

void WildlifeServoControl::enableServoPower() {
    // Enable power to servos (implementation depends on power management circuit)
    servoPowerEnabled = true;
    Serial.println("Servo power enabled");
}

void WildlifeServoControl::disableServoPower() {
    // Disable power to servos for power saving
    servoPowerEnabled = false;
    Serial.println("Servo power disabled");
}

bool WildlifeServoControl::isServoPowerEnabled() const {
    return servoPowerEnabled;
}

bool WildlifeServoControl::isValidPanAngle(int angle) const {
    return (angle >= PAN_MIN_ANGLE && angle <= PAN_MAX_ANGLE);
}

bool WildlifeServoControl::isValidTiltAngle(int angle) const {
    return (angle >= TILT_MIN_ANGLE && angle <= TILT_MAX_ANGLE);
}

void WildlifeServoControl::printCurrentPosition() const {
    Serial.printf("Current position - Pan: %d°, Tilt: %d°\n", 
                  currentPanAngle, currentTiltAngle);
}

int WildlifeServoControl::constrainPanAngle(int angle) const {
    return constrain(angle, PAN_MIN_ANGLE, PAN_MAX_ANGLE);
}

int WildlifeServoControl::constrainTiltAngle(int angle) const {
    return constrain(angle, TILT_MIN_ANGLE, TILT_MAX_ANGLE);
}

void WildlifeServoControl::delayWithStatusUpdate(int ms) {
    // Non-blocking delay that allows for status updates
    unsigned long startTime = millis();
    while (millis() - startTime < ms) {
        // Allow other tasks to run
        yield();
    }
}