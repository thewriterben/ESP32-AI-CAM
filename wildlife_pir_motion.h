/*
  Wildlife PIR Motion Detection Module
  
  Provides enhanced motion detection for wildlife trail camera system
  Integrates with pan/tilt servos for automated wildlife tracking
*/

#include "tcamera_plus_s3_config.h"
#include "wildlife_servo_control.h"
#include <Arduino.h>

#ifndef WILDLIFE_PIR_MOTION_H
#define WILDLIFE_PIR_MOTION_H

struct MotionEvent {
    unsigned long timestamp;
    bool isActive;
    int detectionCount;
    float estimatedDirection;  // Estimated direction of motion in degrees
    int confidence;           // Motion detection confidence (0-100)
};

class WildlifePIRMotion {
private:
    bool pirInitialized;
    bool motionDetected;
    unsigned long lastMotionTime;
    unsigned long motionStartTime;
    int consecutiveDetections;
    
    // Motion filtering parameters
    const unsigned long MOTION_DEBOUNCE_TIME = 100;    // ms
    const unsigned long MOTION_TIMEOUT = 5000;         // 5 seconds
    const int MIN_CONSECUTIVE_DETECTIONS = 3;
    
    // Wildlife-specific parameters
    const unsigned long WILDLIFE_MOTION_DURATION = 2000;  // Expected wildlife motion duration
    const int MOTION_CONFIDENCE_THRESHOLD = 70;
    
    // Statistics
    unsigned long totalDetections;
    unsigned long falsePositives;
    unsigned long confirmedWildlife;
    
    // Motion event history
    static const int MAX_MOTION_EVENTS = 10;
    MotionEvent motionHistory[MAX_MOTION_EVENTS];
    int motionHistoryIndex;

public:
    WildlifePIRMotion();
    ~WildlifePIRMotion();
    
    // Initialization and control
    bool initializePIR();
    void deinitializePIR();
    bool isInitialized() const { return pirInitialized; }
    
    // Motion detection
    bool checkMotion();
    bool isMotionActive() const { return motionDetected; }
    unsigned long getLastMotionTime() const { return lastMotionTime; }
    unsigned long getMotionDuration() const;
    
    // Wildlife-specific detection
    bool isWildlifeMotion() const;
    float estimateMotionDirection();
    int getMotionConfidence() const;
    bool shouldTriggerRecording() const;
    
    // Motion event management
    void recordMotionEvent();
    MotionEvent getLatestMotionEvent() const;
    void clearMotionHistory();
    
    // Filtering and analysis
    bool isValidWildlifePattern() const;
    bool filterWindNoise() const;
    bool filterVegetationMovement() const;
    
    // Integration with servo control
    bool trackMotionWithServos();
    bool scanForMotion();
    
    // Statistics and reporting
    void printMotionStatistics() const;
    float getDetectionAccuracy() const;
    void resetStatistics();
    
    // Configuration
    void setMotionSensitivity(int sensitivity);  // 0-100
    void setConfidenceThreshold(int threshold);
    void enableWindFiltering(bool enable);
    
    // Power management
    void enablePIRPower();
    void disablePIRPower();
    void enterLowPowerMode();
    void exitLowPowerMode();
    
    // Callbacks for motion events
    typedef void (*MotionCallback)(MotionEvent event);
    void setMotionCallback(MotionCallback callback);

private:
    // Internal helper functions
    void updateMotionState();
    bool validateMotionPattern();
    void addToMotionHistory(const MotionEvent& event);
    float calculateMotionDirection();
    int calculateConfidence();
    
    // Filtering algorithms
    bool isConsistentMotion() const;
    bool isHumanLikeMotion() const;
    bool isVehicleMotion() const;
    
    // Configuration variables
    int motionSensitivity;
    int confidenceThreshold;
    bool windFilteringEnabled;
    bool lowPowerMode;
    MotionCallback motionCallback;
    
    // ISR and timing
    static void IRAM_ATTR pirInterruptHandler();
    volatile bool motionInterruptFlag;
    unsigned long lastInterruptTime;
};

// Global PIR motion detection instance
extern WildlifePIRMotion pirMotion;

#endif // WILDLIFE_PIR_MOTION_H