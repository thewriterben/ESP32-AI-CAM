/*
  Wildlife AI Detection and Classification Module
  
  Provides computer vision-based wildlife detection and species classification
  Integrates with camera system and servo control for automated tracking
*/

#include "tcamera_plus_s3_config.h"
#include "wildlife_servo_control.h"
#include "wildlife_pir_motion.h"
#include <Arduino.h>

#ifndef WILDLIFE_AI_DETECTION_H
#define WILDLIFE_AI_DETECTION_H

// Wildlife species classifications
enum WildlifeSpecies {
    SPECIES_UNKNOWN = 0,
    SPECIES_MAMMAL_SMALL = 1,      // Rodents, rabbits, etc.
    SPECIES_MAMMAL_MEDIUM = 2,     // Foxes, cats, raccoons
    SPECIES_MAMMAL_LARGE = 3,      // Deer, bears, wolves
    SPECIES_BIRD_SMALL = 4,        // Songbirds, etc.
    SPECIES_BIRD_MEDIUM = 5,       // Crows, hawks
    SPECIES_BIRD_LARGE = 6,        // Eagles, owls
    SPECIES_REPTILE = 7,           // Snakes, lizards
    SPECIES_HUMAN = 8,             // Human detection
    SPECIES_VEHICLE = 9,           // Vehicle detection
    SPECIES_FALSE_POSITIVE = 10    // Non-wildlife motion
};

// Detection confidence levels
enum ConfidenceLevel {
    CONFIDENCE_VERY_LOW = 0,   // < 30%
    CONFIDENCE_LOW = 1,        // 30-50%
    CONFIDENCE_MEDIUM = 2,     // 50-70%
    CONFIDENCE_HIGH = 3,       // 70-90%
    CONFIDENCE_VERY_HIGH = 4   // > 90%
};

// Wildlife detection result
struct WildlifeDetection {
    uint32_t timestamp;
    WildlifeSpecies species;
    ConfidenceLevel confidence;
    uint8_t confidencePercent;
    
    // Bounding box coordinates (normalized 0-1)
    float x, y, width, height;
    
    // Estimated physical properties
    float estimatedDistance;    // Distance in meters
    float estimatedSize;        // Size in cm
    float movementSpeed;        // Speed in m/s
    float movementDirection;    // Direction in degrees
    
    // Image metadata
    uint16_t imageWidth;
    uint16_t imageHeight;
    uint32_t imageSize;
    
    // Behavioral analysis
    bool isMoving;
    bool isFeeding;
    bool isResting;
    bool isAlert;
    
    // Environmental context
    int8_t lightLevel;          // 0-100
    int8_t temperature;         // Celsius
    uint8_t humidity;           // 0-100
};

// AI model configuration
struct AIModelConfig {
    bool useEdgeAI;             // Use on-device AI vs cloud
    uint8_t detectionThreshold; // Minimum confidence for detection
    uint8_t speciesThreshold;   // Minimum confidence for species ID
    bool enableTracking;        // Enable object tracking
    bool enableBehaviorAnalysis; // Analyze behavior patterns
    uint16_t maxDetections;     // Maximum detections per frame
};

class WildlifeAIDetection {
private:
    bool aiInitialized;
    AIModelConfig config;
    WildlifeDetection lastDetection;
    
    // AI model parameters
    const float INPUT_WIDTH = 416.0;
    const float INPUT_HEIGHT = 416.0;
    const int MAX_OBJECTS = 10;
    const float NMS_THRESHOLD = 0.4;
    
    // Wildlife-specific parameters
    const float MIN_WILDLIFE_SIZE = 0.02;    // Minimum size as fraction of image
    const float MAX_WILDLIFE_SIZE = 0.8;     // Maximum size as fraction of image
    const float MOVEMENT_THRESHOLD = 0.01;   // Minimum movement to consider "moving"
    
    // Detection history for tracking
    static const int DETECTION_HISTORY_SIZE = 10;
    WildlifeDetection detectionHistory[DETECTION_HISTORY_SIZE];
    int historyIndex;
    int historyCount;
    
    // Performance tracking
    unsigned long totalDetections;
    unsigned long wildlifeDetections;
    unsigned long falsePositives;
    unsigned long processingTimeMs;
    
    // Behavioral analysis
    unsigned long lastMovementTime;
    float lastKnownPosition[2];
    float averageMovementSpeed;

public:
    WildlifeAIDetection();
    ~WildlifeAIDetection();
    
    // Initialization and configuration
    bool initializeAI();
    void deinitializeAI();
    bool isInitialized() const { return aiInitialized; }
    
    // Main detection functions
    bool analyzeFrame(uint8_t* imageBuffer, size_t imageSize, WildlifeDetection& result);
    bool detectWildlife(uint8_t* imageBuffer, size_t imageSize);
    bool classifySpecies(uint8_t* imageBuffer, size_t imageSize, WildlifeDetection& result);
    
    // Real-time processing
    bool processLiveVideo();
    bool processMotionEvent(const MotionEvent& motionEvent);
    bool analyzeMovementPattern();
    
    // Species classification
    WildlifeSpecies identifySpecies(const WildlifeDetection& detection);
    ConfidenceLevel calculateConfidence(float rawConfidence);
    bool isWildlife(const WildlifeDetection& detection);
    bool isHuman(const WildlifeDetection& detection);
    bool isVehicle(const WildlifeDetection& detection);
    
    // Behavioral analysis
    bool analyzeBehavior(WildlifeDetection& detection);
    bool detectFeeding(const WildlifeDetection& detection);
    bool detectResting(const WildlifeDetection& detection);
    bool detectAlert(const WildlifeDetection& detection);
    float calculateMovementSpeed(const WildlifeDetection& current, const WildlifeDetection& previous);
    
    // Object tracking
    bool trackObject(WildlifeDetection& detection);
    bool updateTrackingInfo(WildlifeDetection& detection);
    bool predictNextPosition(float& x, float& y);
    
    // Integration with servo control
    bool trackWithServos(const WildlifeDetection& detection);
    bool calculateServoAngles(const WildlifeDetection& detection, int& panAngle, int& tiltAngle);
    bool followTarget(const WildlifeDetection& detection);
    
    // Image preprocessing
    bool preprocessImage(uint8_t* imageBuffer, size_t imageSize);
    bool resizeImage(uint8_t* input, uint8_t* output, int inputW, int inputH, int outputW, int outputH);
    bool normalizeImage(uint8_t* imageBuffer, size_t imageSize);
    bool enhanceContrast(uint8_t* imageBuffer, size_t imageSize);
    
    // Environmental adaptation
    bool adaptToLightConditions(int lightLevel);
    bool adjustForWeather(int temperature, int humidity);
    bool enableNightMode(bool enable);
    
    // Detection history and statistics
    WildlifeDetection getLastDetection() const { return lastDetection; }
    void addToHistory(const WildlifeDetection& detection);
    WildlifeDetection* getDetectionHistory() { return detectionHistory; }
    int getHistoryCount() const { return historyCount; }
    
    // Performance metrics
    void printDetectionStatistics() const;
    float getDetectionAccuracy() const;
    float getAverageProcessingTime() const;
    void resetStatistics();
    
    // Configuration
    void setAIModelConfig(const AIModelConfig& newConfig);
    AIModelConfig getAIModelConfig() const { return config; }
    void setDetectionThreshold(uint8_t threshold);
    void setSpeciesThreshold(uint8_t threshold);
    void enableEdgeAI(bool enable);
    
    // Calibration and training
    bool calibrateDistanceEstimation();
    bool calibrateSizeEstimation();
    bool trainSpeciesClassifier(WildlifeSpecies species, uint8_t* imageBuffer, size_t imageSize);
    
    // Export and data management
    bool saveDetectionData(const WildlifeDetection& detection);
    bool exportDetectionHistory();
    bool createThumbnail(uint8_t* imageBuffer, size_t imageSize, uint8_t* thumbnail, size_t& thumbnailSize);
    
    // Callbacks for detection events
    typedef void (*WildlifeDetectionCallback)(const WildlifeDetection& detection);
    typedef void (*SpeciesIdentificationCallback)(WildlifeSpecies species, float confidence);
    typedef void (*BehaviorAnalysisCallback)(const WildlifeDetection& detection, const char* behavior);
    
    void setWildlifeDetectionCallback(WildlifeDetectionCallback callback);
    void setSpeciesIdentificationCallback(SpeciesIdentificationCallback callback);
    void setBehaviorAnalysisCallback(BehaviorAnalysisCallback callback);

private:
    // Internal AI processing functions
    bool runInference(uint8_t* imageBuffer, size_t imageSize, float* output);
    bool postProcessDetections(float* modelOutput, WildlifeDetection* detections, int& numDetections);
    bool applyNonMaxSuppression(WildlifeDetection* detections, int& numDetections);
    
    // Utility functions
    float calculateIoU(const WildlifeDetection& det1, const WildlifeDetection& det2);
    float calculateDistance(float x1, float y1, float x2, float y2);
    bool isValidDetection(const WildlifeDetection& detection);
    
    // Environmental context
    void updateEnvironmentalContext(WildlifeDetection& detection);
    
    // Callback functions
    WildlifeDetectionCallback wildlifeDetectionCallback;
    SpeciesIdentificationCallback speciesIdentificationCallback;
    BehaviorAnalysisCallback behaviorAnalysisCallback;
};

// Global AI detection instance
extern WildlifeAIDetection aiDetection;

#endif // WILDLIFE_AI_DETECTION_H