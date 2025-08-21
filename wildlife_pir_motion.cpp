/*
  Wildlife PIR Motion Detection Implementation
  
  Enhanced motion detection for wildlife trail camera system
*/

#include "wildlife_pir_motion.h"
#include "tcamera_plus_s3_config.h"

// Global PIR motion detection instance
WildlifePIRMotion pirMotion;

// Static member for ISR
volatile bool WildlifePIRMotion::motionInterruptFlag = false;

WildlifePIRMotion::WildlifePIRMotion() {
    pirInitialized = false;
    motionDetected = false;
    lastMotionTime = 0;
    motionStartTime = 0;
    consecutiveDetections = 0;
    totalDetections = 0;
    falsePositives = 0;
    confirmedWildlife = 0;
    motionHistoryIndex = 0;
    motionSensitivity = 75;
    confidenceThreshold = MOTION_CONFIDENCE_THRESHOLD;
    windFilteringEnabled = true;
    lowPowerMode = false;
    motionCallback = nullptr;
    motionInterruptFlag = false;
    lastInterruptTime = 0;
    
    // Initialize motion history
    for (int i = 0; i < MAX_MOTION_EVENTS; i++) {
        motionHistory[i] = {0, false, 0, 0.0, 0};
    }
}

WildlifePIRMotion::~WildlifePIRMotion() {
    deinitializePIR();
}

bool WildlifePIRMotion::initializePIR() {
    if (pirInitialized) {
        return true;
    }
    
    Serial.println("Initializing PIR motion detection...");
    
    // Configure PIR pin as input with pull-down
    pinMode(PIR_MOTION_PIN, INPUT_PULLDOWN);
    
    // Enable PIR power
    enablePIRPower();
    
    // Allow PIR sensor to stabilize
    Serial.println("PIR sensor stabilizing...");
    delay(30000); // PIR sensors typically need 30 seconds to stabilize
    
    // Attach interrupt for motion detection
    attachInterrupt(digitalPinToInterrupt(PIR_MOTION_PIN), 
                   pirInterruptHandler, RISING);
    
    pirInitialized = true;
    Serial.println("PIR motion detection initialized successfully");
    
    return true;
}

void WildlifePIRMotion::deinitializePIR() {
    if (!pirInitialized) {
        return;
    }
    
    Serial.println("Deinitializing PIR motion detection...");
    
    detachInterrupt(digitalPinToInterrupt(PIR_MOTION_PIN));
    disablePIRPower();
    
    pirInitialized = false;
    Serial.println("PIR motion detection deinitialized");
}

bool WildlifePIRMotion::checkMotion() {
    if (!pirInitialized) {
        return false;
    }
    
    bool currentMotion = digitalRead(PIR_MOTION_PIN) == HIGH || motionInterruptFlag;
    
    if (motionInterruptFlag) {
        motionInterruptFlag = false; // Clear interrupt flag
    }
    
    unsigned long currentTime = millis();
    
    // Debounce motion detection
    if (currentTime - lastInterruptTime < MOTION_DEBOUNCE_TIME) {
        return motionDetected;
    }
    
    if (currentMotion && !motionDetected) {
        // Motion started
        motionDetected = true;
        motionStartTime = currentTime;
        lastMotionTime = currentTime;
        consecutiveDetections = 1;
        totalDetections++;
        
        Serial.println("Motion detected!");
        recordMotionEvent();
        
        // Trigger callback if set
        if (motionCallback) {
            MotionEvent event = getLatestMotionEvent();
            motionCallback(event);
        }
        
    } else if (currentMotion && motionDetected) {
        // Motion continues
        lastMotionTime = currentTime;
        consecutiveDetections++;
        
    } else if (!currentMotion && motionDetected) {
        // Check if motion timeout reached
        if (currentTime - lastMotionTime > MOTION_TIMEOUT) {
            motionDetected = false;
            Serial.println("Motion ended");
        }
    }
    
    return motionDetected;
}

unsigned long WildlifePIRMotion::getMotionDuration() const {
    if (!motionDetected) {
        return 0;
    }
    return millis() - motionStartTime;
}

bool WildlifePIRMotion::isWildlifeMotion() const {
    if (!motionDetected) {
        return false;
    }
    
    // Check motion duration - wildlife typically moves for 2-10 seconds
    unsigned long duration = getMotionDuration();
    if (duration < 500 || duration > 30000) {
        return false; // Too short (noise) or too long (human activity)
    }
    
    // Check consecutive detections for consistent motion
    if (consecutiveDetections < MIN_CONSECUTIVE_DETECTIONS) {
        return false;
    }
    
    // Apply wildlife-specific filters
    if (!isValidWildlifePattern()) {
        return false;
    }
    
    if (windFilteringEnabled && filterWindNoise()) {
        return false;
    }
    
    return true;
}

float WildlifePIRMotion::estimateMotionDirection() {
    // Simple direction estimation based on motion pattern
    // In a real implementation, this would use multiple PIR sensors
    // For now, return a placeholder value
    return calculateMotionDirection();
}

int WildlifePIRMotion::getMotionConfidence() const {
    return calculateConfidence();
}

bool WildlifePIRMotion::shouldTriggerRecording() const {
    if (!isWildlifeMotion()) {
        return false;
    }
    
    return getMotionConfidence() >= confidenceThreshold;
}

void WildlifePIRMotion::recordMotionEvent() {
    MotionEvent event;
    event.timestamp = millis();
    event.isActive = motionDetected;
    event.detectionCount = consecutiveDetections;
    event.estimatedDirection = estimateMotionDirection();
    event.confidence = getMotionConfidence();
    
    addToMotionHistory(event);
}

MotionEvent WildlifePIRMotion::getLatestMotionEvent() const {
    int latestIndex = (motionHistoryIndex - 1 + MAX_MOTION_EVENTS) % MAX_MOTION_EVENTS;
    return motionHistory[latestIndex];
}

void WildlifePIRMotion::clearMotionHistory() {
    for (int i = 0; i < MAX_MOTION_EVENTS; i++) {
        motionHistory[i] = {0, false, 0, 0.0, 0};
    }
    motionHistoryIndex = 0;
}

bool WildlifePIRMotion::isValidWildlifePattern() const {
    // Analyze motion pattern for wildlife characteristics
    unsigned long duration = getMotionDuration();
    
    // Wildlife motion characteristics:
    // - Consistent movement for 2-10 seconds
    // - Not too erratic (which might indicate wind)
    // - Not perfectly regular (which might indicate mechanical)
    
    if (duration >= WILDLIFE_MOTION_DURATION && 
        consecutiveDetections >= MIN_CONSECUTIVE_DETECTIONS &&
        !isHumanLikeMotion() && 
        !isVehicleMotion()) {
        return true;
    }
    
    return false;
}

bool WildlifePIRMotion::filterWindNoise() const {
    // Check for wind-induced false positives
    // Wind typically causes very brief, irregular motion
    unsigned long duration = getMotionDuration();
    
    if (duration < 1000 && consecutiveDetections < 3) {
        return true; // Likely wind noise
    }
    
    return false;
}

bool WildlifePIRMotion::filterVegetationMovement() const {
    // Filter out vegetation movement
    // Vegetation tends to cause repetitive, rhythmic motion
    return false; // Placeholder implementation
}

bool WildlifePIRMotion::trackMotionWithServos() {
    if (!motionDetected || !servoControl.isInitialized()) {
        return false;
    }
    
    Serial.println("Tracking motion with servos");
    
    // Estimate direction and track
    float direction = estimateMotionDirection();
    
    // Convert direction to servo angles
    int panAngle = (int)direction;
    int tiltAngle = 0; // Assume ground level for wildlife
    
    return servoControl.trackTarget(panAngle, tiltAngle, true);
}

bool WildlifePIRMotion::scanForMotion() {
    if (!servoControl.isInitialized()) {
        return false;
    }
    
    Serial.println("Scanning for motion");
    
    // Perform wildlife-optimized scan
    return servoControl.performWildlifeScan();
}

void WildlifePIRMotion::printMotionStatistics() const {
    Serial.println("=== Motion Detection Statistics ===");
    Serial.printf("Total detections: %lu\n", totalDetections);
    Serial.printf("False positives: %lu\n", falsePositives);
    Serial.printf("Confirmed wildlife: %lu\n", confirmedWildlife);
    Serial.printf("Detection accuracy: %.1f%%\n", getDetectionAccuracy());
    Serial.printf("Current sensitivity: %d\n", motionSensitivity);
    Serial.printf("Confidence threshold: %d\n", confidenceThreshold);
    Serial.printf("Wind filtering: %s\n", windFilteringEnabled ? "Enabled" : "Disabled");
}

float WildlifePIRMotion::getDetectionAccuracy() const {
    if (totalDetections == 0) {
        return 0.0;
    }
    return (float)(totalDetections - falsePositives) / totalDetections * 100.0;
}

void WildlifePIRMotion::resetStatistics() {
    totalDetections = 0;
    falsePositives = 0;
    confirmedWildlife = 0;
    clearMotionHistory();
}

void WildlifePIRMotion::setMotionSensitivity(int sensitivity) {
    motionSensitivity = constrain(sensitivity, 0, 100);
    Serial.printf("Motion sensitivity set to: %d\n", motionSensitivity);
}

void WildlifePIRMotion::setConfidenceThreshold(int threshold) {
    confidenceThreshold = constrain(threshold, 0, 100);
    Serial.printf("Confidence threshold set to: %d\n", confidenceThreshold);
}

void WildlifePIRMotion::enableWindFiltering(bool enable) {
    windFilteringEnabled = enable;
    Serial.printf("Wind filtering: %s\n", enable ? "Enabled" : "Disabled");
}

void WildlifePIRMotion::enablePIRPower() {
    // Enable power to PIR sensor
    Serial.println("PIR power enabled");
}

void WildlifePIRMotion::disablePIRPower() {
    // Disable power to PIR sensor for power saving
    Serial.println("PIR power disabled");
}

void WildlifePIRMotion::enterLowPowerMode() {
    lowPowerMode = true;
    Serial.println("PIR entering low power mode");
}

void WildlifePIRMotion::exitLowPowerMode() {
    lowPowerMode = false;
    Serial.println("PIR exiting low power mode");
}

void WildlifePIRMotion::setMotionCallback(MotionCallback callback) {
    motionCallback = callback;
}

// Private helper functions

void WildlifePIRMotion::addToMotionHistory(const MotionEvent& event) {
    motionHistory[motionHistoryIndex] = event;
    motionHistoryIndex = (motionHistoryIndex + 1) % MAX_MOTION_EVENTS;
}

float WildlifePIRMotion::calculateMotionDirection() {
    // Placeholder implementation for direction calculation
    // In a real system, this would use multiple PIR sensors or camera analysis
    return 0.0; // Center direction
}

int WildlifePIRMotion::calculateConfidence() {
    int confidence = 50; // Base confidence
    
    // Increase confidence based on motion duration
    unsigned long duration = getMotionDuration();
    if (duration > WILDLIFE_MOTION_DURATION) {
        confidence += 20;
    }
    
    // Increase confidence based on consecutive detections
    if (consecutiveDetections > MIN_CONSECUTIVE_DETECTIONS) {
        confidence += 15;
    }
    
    // Adjust based on motion pattern validation
    if (isValidWildlifePattern()) {
        confidence += 15;
    }
    
    return constrain(confidence, 0, 100);
}

bool WildlifePIRMotion::isConsistentMotion() const {
    return consecutiveDetections >= MIN_CONSECUTIVE_DETECTIONS;
}

bool WildlifePIRMotion::isHumanLikeMotion() const {
    // Human motion is typically longer duration and more regular
    unsigned long duration = getMotionDuration();
    return duration > 30000; // More than 30 seconds suggests human activity
}

bool WildlifePIRMotion::isVehicleMotion() const {
    // Vehicle motion is typically very brief but high intensity
    unsigned long duration = getMotionDuration();
    return duration < 500 && consecutiveDetections > 10;
}

// ISR for PIR interrupt
void IRAM_ATTR WildlifePIRMotion::pirInterruptHandler() {
    unsigned long currentTime = millis();
    static unsigned long lastInterrupt = 0;
    
    // Debounce in ISR
    if (currentTime - lastInterrupt > 50) {
        motionInterruptFlag = true;
        lastInterrupt = currentTime;
    }
}