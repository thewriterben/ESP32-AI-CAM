/*
  Wildlife Trail Camera Servo Control Module
  
  Provides pan/tilt functionality for wildlife tracking using SG90 micro servos
  Supports 270° pan and 180° tilt coverage for comprehensive area monitoring
*/

#include "tcamera_plus_s3_config.h"
#include <ESP32Servo.h>
#include <Arduino.h>

#ifndef WILDLIFE_SERVO_CONTROL_H
#define WILDLIFE_SERVO_CONTROL_H

class WildlifeServoControl {
private:
    Servo panServo;
    Servo tiltServo;
    int currentPanAngle;
    int currentTiltAngle;
    bool servosInitialized;
    
    // Servo position limits
    const int PAN_MIN_ANGLE = -135;   // -135° from center
    const int PAN_MAX_ANGLE = 135;    // +135° from center (270° total)
    const int TILT_MIN_ANGLE = -90;   // -90° from horizontal
    const int TILT_MAX_ANGLE = 90;    // +90° from horizontal (180° total)
    
    // Movement parameters
    const int MOVEMENT_DELAY = 20;    // Delay between servo steps (ms)
    const int STEP_SIZE = 2;          // Degrees per step for smooth movement

public:
    WildlifeServoControl();
    ~WildlifeServoControl();
    
    // Initialization and control
    bool initializeServos();
    void deinitializeServos();
    bool isInitialized() const { return servosInitialized; }
    
    // Basic positioning
    bool setPanAngle(int angle);
    bool setTiltAngle(int angle);
    bool setPosition(int panAngle, int tiltAngle);
    
    // Smooth movement functions
    bool smoothPanTo(int targetAngle);
    bool smoothTiltTo(int targetAngle);
    bool smoothMoveTo(int panAngle, int tiltAngle);
    
    // Current position getters
    int getCurrentPanAngle() const { return currentPanAngle; }
    int getCurrentTiltAngle() const { return currentTiltAngle; }
    
    // Preset positions for wildlife monitoring
    bool moveToHomePosition();        // Center position (0°, 0°)
    bool moveToScanPosition();        // Elevated scan position
    bool moveToRestPosition();        // Power-saving position
    
    // Automated scanning patterns
    bool performHorizontalScan(int startAngle = -90, int endAngle = 90);
    bool performVerticalScan(int startAngle = -45, int endAngle = 45);
    bool performGridScan();           // Complete area scan in grid pattern
    bool performWildlifeScan();       // Optimized scan for wildlife detection
    
    // Wildlife tracking functions
    bool trackTarget(int panAngle, int tiltAngle, bool smooth = true);
    bool sweepArea(int centerPan, int centerTilt, int sweepRadius);
    
    // Utility functions
    bool isValidPanAngle(int angle) const;
    bool isValidTiltAngle(int angle) const;
    void printCurrentPosition() const;
    
    // Power management
    void enableServoPower();
    void disableServoPower();
    bool isServoPowerEnabled() const;
    
private:
    // Internal helper functions
    int constrainPanAngle(int angle) const;
    int constrainTiltAngle(int angle) const;
    void delayWithStatusUpdate(int ms);
    bool servoPowerEnabled;
};

// Global servo control instance
extern WildlifeServoControl servoControl;

#endif // WILDLIFE_SERVO_CONTROL_H