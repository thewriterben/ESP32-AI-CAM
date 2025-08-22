/*
  Wildlife AI Detection and Classification Implementation
  
  Computer vision-based wildlife detection and species classification
*/

#include "wildlife_ai_detection.h"
#include "tcamera_plus_s3_config.h"

// Global AI detection instance
WildlifeAIDetection aiDetection;

WildlifeAIDetection::WildlifeAIDetection() {
    aiInitialized = false;
    historyIndex = 0;
    historyCount = 0;
    totalDetections = 0;
    wildlifeDetections = 0;
    falsePositives = 0;
    processingTimeMs = 0;
    lastMovementTime = 0;
    lastKnownPosition[0] = 0.0;
    lastKnownPosition[1] = 0.0;
    averageMovementSpeed = 0.0;
    
    // Default AI configuration
    config.useEdgeAI = true;
    config.detectionThreshold = 70;
    config.speciesThreshold = 60;
    config.enableTracking = true;
    config.enableBehaviorAnalysis = true;
    config.maxDetections = 5;
    
    // Initialize detection history
    for (int i = 0; i < DETECTION_HISTORY_SIZE; i++) {
        detectionHistory[i] = {0};
    }
    
    // Initialize callbacks
    wildlifeDetectionCallback = nullptr;
    speciesIdentificationCallback = nullptr;
    behaviorAnalysisCallback = nullptr;
    
    // Initialize last detection
    lastDetection = {0};
}

WildlifeAIDetection::~WildlifeAIDetection() {
    deinitializeAI();
}

bool WildlifeAIDetection::initializeAI() {
    if (aiInitialized) {
        return true;
    }
    
    Serial.println("Initializing Wildlife AI Detection System...");
    
    // Initialize AI model (placeholder for actual model loading)
    // In a real implementation, this would load a trained model
    // such as YOLO, MobileNet, or a custom wildlife detection model
    
    Serial.println("Loading wildlife detection model...");
    delay(2000); // Simulate model loading time
    
    // Initialize image processing buffers
    // (Implementation would allocate memory for image processing)
    
    aiInitialized = true;
    Serial.println("Wildlife AI Detection System initialized successfully");
    
    return true;
}

void WildlifeAIDetection::deinitializeAI() {
    if (!aiInitialized) {
        return;
    }
    
    Serial.println("Deinitializing AI Detection System...");
    
    // Cleanup model and buffers
    // (Implementation would free allocated memory)
    
    aiInitialized = false;
    Serial.println("AI Detection System deinitialized");
}

bool WildlifeAIDetection::analyzeFrame(uint8_t* imageBuffer, size_t imageSize, WildlifeDetection& result) {
    if (!aiInitialized) {
        Serial.println("AI not initialized");
        return false;
    }
    
    unsigned long startTime = millis();
    
    // Preprocess image
    if (!preprocessImage(imageBuffer, imageSize)) {
        Serial.println("Image preprocessing failed");
        return false;
    }
    
    // Run wildlife detection
    if (!detectWildlife(imageBuffer, imageSize)) {
        // No wildlife detected
        result.species = SPECIES_UNKNOWN;
        result.confidence = CONFIDENCE_VERY_LOW;
        result.confidencePercent = 0;
        return false;
    }
    
    // Classify species
    if (!classifySpecies(imageBuffer, imageSize, result)) {
        Serial.println("Species classification failed");
        return false;
    }
    
    // Analyze behavior if enabled
    if (config.enableBehaviorAnalysis) {
        analyzeBehavior(result);
    }
    
    // Update tracking information if enabled
    if (config.enableTracking) {
        trackObject(result);
    }
    
    // Update environmental context
    updateEnvironmentalContext(result);
    
    // Add to history
    addToHistory(result);
    lastDetection = result;
    
    // Update statistics
    totalDetections++;
    if (result.species != SPECIES_UNKNOWN && result.species != SPECIES_FALSE_POSITIVE) {
        wildlifeDetections++;
    } else if (result.species == SPECIES_FALSE_POSITIVE) {
        falsePositives++;
    }
    
    // Update processing time
    unsigned long processingTime = millis() - startTime;
    processingTimeMs = (processingTimeMs * 9 + processingTime) / 10; // Moving average
    
    Serial.printf("Wildlife detected: Species=%d, Confidence=%d%%, Processing=%lums\n", 
                  result.species, result.confidencePercent, processingTime);
    
    // Trigger callbacks
    if (wildlifeDetectionCallback && result.species != SPECIES_UNKNOWN) {
        wildlifeDetectionCallback(result);
    }
    
    if (speciesIdentificationCallback && result.species != SPECIES_UNKNOWN) {
        speciesIdentificationCallback(result.species, result.confidencePercent);
    }
    
    return true;
}

bool WildlifeAIDetection::detectWildlife(uint8_t* imageBuffer, size_t imageSize) {
    // Simplified wildlife detection algorithm
    // In a real implementation, this would use a trained neural network
    
    // For demonstration, use basic motion-based detection
    // This is a placeholder that simulates wildlife detection
    
    // Simulate processing time
    delay(50);
    
    // Return true if motion was detected and it looks like wildlife
    // (This would be replaced with actual AI inference)
    return pirMotion.isMotionActive() && pirMotion.isWildlifeMotion();
}

bool WildlifeAIDetection::classifySpecies(uint8_t* imageBuffer, size_t imageSize, WildlifeDetection& result) {
    // Simplified species classification
    // In a real implementation, this would use a species classification model
    
    result.timestamp = millis();
    
    // Simulate species classification based on various factors
    // This is a placeholder implementation
    
    // Analyze image characteristics (simulated)
    int brightness = 128; // Placeholder
    int motionIntensity = pirMotion.getMotionConfidence();
    
    // Basic heuristic classification (replace with actual AI model)
    if (motionIntensity > 80) {
        // High motion intensity suggests larger animal
        result.species = SPECIES_MAMMAL_LARGE;
        result.confidencePercent = 75;
        result.estimatedSize = 100.0; // cm
        result.estimatedDistance = 8.0; // meters
    } else if (motionIntensity > 50) {
        // Medium motion suggests medium-sized animal
        result.species = SPECIES_MAMMAL_MEDIUM;
        result.confidencePercent = 65;
        result.estimatedSize = 40.0; // cm
        result.estimatedDistance = 5.0; // meters
    } else if (motionIntensity > 20) {
        // Low motion could be small animal or bird
        if (brightness > 100) {
            result.species = SPECIES_BIRD_SMALL;
            result.confidencePercent = 55;
            result.estimatedSize = 15.0; // cm
            result.estimatedDistance = 3.0; // meters
        } else {
            result.species = SPECIES_MAMMAL_SMALL;
            result.confidencePercent = 50;
            result.estimatedSize = 20.0; // cm
            result.estimatedDistance = 4.0; // meters
        }
    } else {
        result.species = SPECIES_UNKNOWN;
        result.confidencePercent = 10;
    }
    
    result.confidence = calculateConfidence(result.confidencePercent);
    
    // Set basic bounding box (simulated)
    result.x = 0.3;
    result.y = 0.4;
    result.width = 0.4;
    result.height = 0.3;
    
    // Set image metadata
    result.imageWidth = 640;  // Placeholder
    result.imageHeight = 480; // Placeholder
    result.imageSize = imageSize;
    
    return result.confidencePercent >= config.speciesThreshold;
}

bool WildlifeAIDetection::processMotionEvent(const MotionEvent& motionEvent) {
    if (!aiInitialized) {
        return false;
    }
    
    Serial.println("Processing motion event for AI analysis");
    
    // This would integrate with the camera to capture an image
    // and then analyze it for wildlife
    
    // For now, simulate the process
    WildlifeDetection detection;
    detection.timestamp = motionEvent.timestamp;
    detection.species = SPECIES_MAMMAL_MEDIUM; // Placeholder
    detection.confidencePercent = motionEvent.confidence;
    detection.confidence = calculateConfidence(detection.confidencePercent);
    
    // Trigger servo tracking if wildlife detected
    if (detection.species != SPECIES_UNKNOWN && config.enableTracking) {
        trackWithServos(detection);
    }
    
    return true;
}

bool WildlifeAIDetection::analyzeBehavior(WildlifeDetection& detection) {
    // Simple behavioral analysis based on movement patterns
    
    // Determine if animal is moving
    if (historyCount > 0) {
        WildlifeDetection& prev = detectionHistory[(historyIndex - 1 + DETECTION_HISTORY_SIZE) % DETECTION_HISTORY_SIZE];
        float displacement = calculateDistance(detection.x, detection.y, prev.x, prev.y);
        detection.isMoving = displacement > MOVEMENT_THRESHOLD;
        
        if (detection.isMoving) {
            detection.movementSpeed = calculateMovementSpeed(detection, prev);
            lastMovementTime = detection.timestamp;
        }
    }
    
    // Analyze feeding behavior (simplified)
    detection.isFeeding = detectFeeding(detection);
    
    // Analyze resting behavior
    detection.isResting = detectResting(detection);
    
    // Analyze alert behavior
    detection.isAlert = detectAlert(detection);
    
    return true;
}

bool WildlifeAIDetection::detectFeeding(const WildlifeDetection& detection) {
    // Simple heuristic: low movement for extended period suggests feeding
    if (!detection.isMoving && historyCount > 3) {
        int stationaryCount = 0;
        for (int i = 0; i < min(historyCount, 5); i++) {
            int index = (historyIndex - i - 1 + DETECTION_HISTORY_SIZE) % DETECTION_HISTORY_SIZE;
            if (!detectionHistory[index].isMoving) {
                stationaryCount++;
            }
        }
        return stationaryCount >= 3;
    }
    return false;
}

bool WildlifeAIDetection::detectResting(const WildlifeDetection& detection) {
    // Resting: stationary for long period with low alert level
    return !detection.isMoving && !detection.isAlert && 
           (millis() - lastMovementTime) > 30000; // 30 seconds stationary
}

bool WildlifeAIDetection::detectAlert(const WildlifeDetection& detection) {
    // Alert behavior: recent sudden movement or position changes
    if (historyCount > 0) {
        WildlifeDetection& prev = detectionHistory[(historyIndex - 1 + DETECTION_HISTORY_SIZE) % DETECTION_HISTORY_SIZE];
        float displacement = calculateDistance(detection.x, detection.y, prev.x, prev.y);
        return displacement > 0.05; // Significant position change
    }
    return false;
}

float WildlifeAIDetection::calculateMovementSpeed(const WildlifeDetection& current, const WildlifeDetection& previous) {
    float distance = calculateDistance(current.x, current.y, previous.x, previous.y);
    float timeDiff = (current.timestamp - previous.timestamp) / 1000.0; // seconds
    
    if (timeDiff > 0) {
        // Convert from image coordinates to real-world speed (simplified)
        float realWorldDistance = distance * current.estimatedDistance; // rough approximation
        return realWorldDistance / timeDiff; // m/s
    }
    
    return 0.0;
}

bool WildlifeAIDetection::trackWithServos(const WildlifeDetection& detection) {
    if (!servoControl.isInitialized()) {
        return false;
    }
    
    int panAngle, tiltAngle;
    if (!calculateServoAngles(detection, panAngle, tiltAngle)) {
        return false;
    }
    
    Serial.printf("Tracking wildlife with servos: Pan=%d°, Tilt=%d°\n", panAngle, tiltAngle);
    
    return servoControl.trackTarget(panAngle, tiltAngle, true);
}

bool WildlifeAIDetection::calculateServoAngles(const WildlifeDetection& detection, int& panAngle, int& tiltAngle) {
    // Convert image coordinates to servo angles
    // Image coordinates are normalized (0-1), center is (0.5, 0.5)
    
    // Calculate pan angle (horizontal movement)
    float panOffset = (detection.x + detection.width/2.0) - 0.5; // Offset from center
    panAngle = (int)(panOffset * 180.0); // Convert to degrees
    
    // Calculate tilt angle (vertical movement)
    float tiltOffset = 0.5 - (detection.y + detection.height/2.0); // Inverted Y
    tiltAngle = (int)(tiltOffset * 90.0); // Convert to degrees
    
    // Constrain angles to servo limits
    panAngle = constrain(panAngle, -135, 135);
    tiltAngle = constrain(tiltAngle, -90, 90);
    
    return true;
}

bool WildlifeAIDetection::preprocessImage(uint8_t* imageBuffer, size_t imageSize) {
    // Basic image preprocessing
    // In a real implementation, this would include:
    // - Resizing to model input size
    // - Normalization
    // - Color space conversion
    // - Noise reduction
    
    // For now, just simulate preprocessing
    delay(10);
    
    return true;
}

ConfidenceLevel WildlifeAIDetection::calculateConfidence(float rawConfidence) {
    if (rawConfidence >= 90) return CONFIDENCE_VERY_HIGH;
    else if (rawConfidence >= 70) return CONFIDENCE_HIGH;
    else if (rawConfidence >= 50) return CONFIDENCE_MEDIUM;
    else if (rawConfidence >= 30) return CONFIDENCE_LOW;
    else return CONFIDENCE_VERY_LOW;
}

bool WildlifeAIDetection::isWildlife(const WildlifeDetection& detection) {
    return detection.species >= SPECIES_MAMMAL_SMALL && detection.species <= SPECIES_REPTILE;
}

bool WildlifeAIDetection::isHuman(const WildlifeDetection& detection) {
    return detection.species == SPECIES_HUMAN;
}

bool WildlifeAIDetection::isVehicle(const WildlifeDetection& detection) {
    return detection.species == SPECIES_VEHICLE;
}

void WildlifeAIDetection::addToHistory(const WildlifeDetection& detection) {
    detectionHistory[historyIndex] = detection;
    historyIndex = (historyIndex + 1) % DETECTION_HISTORY_SIZE;
    if (historyCount < DETECTION_HISTORY_SIZE) {
        historyCount++;
    }
}

void WildlifeAIDetection::printDetectionStatistics() const {
    Serial.println("=== Wildlife AI Detection Statistics ===");
    Serial.printf("Total detections: %lu\n", totalDetections);
    Serial.printf("Wildlife detections: %lu\n", wildlifeDetections);
    Serial.printf("False positives: %lu\n", falsePositives);
    Serial.printf("Detection accuracy: %.1f%%\n", getDetectionAccuracy());
    Serial.printf("Average processing time: %.1f ms\n", getAverageProcessingTime());
    Serial.printf("Detection threshold: %d%%\n", config.detectionThreshold);
    Serial.printf("Species threshold: %d%%\n", config.speciesThreshold);
}

float WildlifeAIDetection::getDetectionAccuracy() const {
    if (totalDetections == 0) {
        return 0.0;
    }
    return (float)(totalDetections - falsePositives) / totalDetections * 100.0;
}

float WildlifeAIDetection::getAverageProcessingTime() const {
    return (float)processingTimeMs;
}

void WildlifeAIDetection::resetStatistics() {
    totalDetections = 0;
    wildlifeDetections = 0;
    falsePositives = 0;
    processingTimeMs = 0;
    historyCount = 0;
    historyIndex = 0;
}

void WildlifeAIDetection::setAIModelConfig(const AIModelConfig& newConfig) {
    config = newConfig;
    Serial.println("AI model configuration updated");
}

void WildlifeAIDetection::setDetectionThreshold(uint8_t threshold) {
    config.detectionThreshold = constrain(threshold, 0, 100);
    Serial.printf("Detection threshold set to: %d%%\n", config.detectionThreshold);
}

void WildlifeAIDetection::setSpeciesThreshold(uint8_t threshold) {
    config.speciesThreshold = constrain(threshold, 0, 100);
    Serial.printf("Species threshold set to: %d%%\n", config.speciesThreshold);
}

void WildlifeAIDetection::enableEdgeAI(bool enable) {
    config.useEdgeAI = enable;
    Serial.printf("Edge AI: %s\n", enable ? "Enabled" : "Disabled");
}

void WildlifeAIDetection::updateEnvironmentalContext(WildlifeDetection& detection) {
    // Update environmental context from sensors
    // This would read from actual environmental sensors
    detection.lightLevel = 75; // Placeholder
    detection.temperature = 20; // Placeholder
    detection.humidity = 60;    // Placeholder
}

float WildlifeAIDetection::calculateDistance(float x1, float y1, float x2, float y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

bool WildlifeAIDetection::isValidDetection(const WildlifeDetection& detection) {
    return detection.width >= MIN_WILDLIFE_SIZE && 
           detection.width <= MAX_WILDLIFE_SIZE &&
           detection.height >= MIN_WILDLIFE_SIZE && 
           detection.height <= MAX_WILDLIFE_SIZE &&
           detection.confidencePercent >= config.detectionThreshold;
}

// Callback setters
void WildlifeAIDetection::setWildlifeDetectionCallback(WildlifeDetectionCallback callback) {
    wildlifeDetectionCallback = callback;
}

void WildlifeAIDetection::setSpeciesIdentificationCallback(SpeciesIdentificationCallback callback) {
    speciesIdentificationCallback = callback;
}

void WildlifeAIDetection::setBehaviorAnalysisCallback(BehaviorAnalysisCallback callback) {
    behaviorAnalysisCallback = callback;
}