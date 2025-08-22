/*
  Wildlife Trail Camera LoRa Communication Module
  
  Provides long-range communication between trail cameras
  Supports mesh networking and status reporting
*/

#include "tcamera_plus_s3_config.h"
#include <LoRa.h>
#include <Arduino.h>

#ifndef WILDLIFE_LORA_COMM_H
#define WILDLIFE_LORA_COMM_H

// Message types for LoRa communication
enum LoRaMessageType {
    LORA_MSG_HEARTBEAT = 0x01,
    LORA_MSG_MOTION_ALERT = 0x02,
    LORA_MSG_WILDLIFE_DETECTED = 0x03,
    LORA_MSG_STATUS_REQUEST = 0x04,
    LORA_MSG_STATUS_RESPONSE = 0x05,
    LORA_MSG_CAMERA_CONTROL = 0x06,
    LORA_MSG_DATA_SYNC = 0x07,
    LORA_MSG_EMERGENCY = 0x08,
    LORA_MSG_BATTERY_LOW = 0x09,
    LORA_MSG_NETWORK_JOIN = 0x0A
};

// LoRa message structure
struct LoRaMessage {
    uint8_t messageType;
    uint8_t sourceId;
    uint8_t destId;      // 0xFF for broadcast
    uint8_t sequenceNum;
    uint8_t payloadLength;
    uint8_t payload[200]; // Max payload size
    uint16_t checksum;
};

// Wildlife detection data structure
struct WildlifeDetection {
    uint32_t timestamp;
    float latitude;
    float longitude;
    uint8_t speciesType;     // 0=unknown, 1=mammal, 2=bird, 3=other
    uint8_t confidence;      // 0-100
    uint16_t imageSize;
    float batteryVoltage;
    int16_t panAngle;
    int16_t tiltAngle;
};

// Camera status structure
struct CameraStatus {
    uint8_t cameraId;
    uint32_t timestamp;
    float batteryVoltage;
    float solarVoltage;
    int16_t temperature;
    uint8_t humidity;
    uint16_t lightLevel;
    uint8_t motionEvents;
    uint8_t wildlifeDetections;
    int8_t rssi;
    uint8_t errorFlags;
};

class WildlifeLoRaComm {
private:
    bool loraInitialized;
    uint8_t cameraId;
    uint8_t sequenceNumber;
    unsigned long lastHeartbeat;
    unsigned long lastMessageReceived;
    
    // Network parameters
    const unsigned long HEARTBEAT_INTERVAL = 300000;    // 5 minutes
    const unsigned long MESSAGE_TIMEOUT = 10000;        // 10 seconds
    const int MAX_RETRIES = 3;
    const int RSSI_THRESHOLD = -120;                     // Minimum RSSI
    
    // Mesh networking
    static const int MAX_KNOWN_CAMERAS = 10;
    CameraStatus knownCameras[MAX_KNOWN_CAMERAS];
    int knownCameraCount;
    
    // Message queue
    static const int MESSAGE_QUEUE_SIZE = 20;
    LoRaMessage messageQueue[MESSAGE_QUEUE_SIZE];
    int queueHead;
    int queueTail;
    int queueCount;

public:
    WildlifeLoRaComm(uint8_t camId = 1);
    ~WildlifeLoRaComm();
    
    // Initialization and control
    bool initializeLoRa();
    void deinitializeLoRa();
    bool isInitialized() const { return loraInitialized; }
    
    // Basic communication
    bool sendMessage(const LoRaMessage& message);
    bool receiveMessage(LoRaMessage& message);
    bool sendBroadcast(uint8_t messageType, const uint8_t* payload, uint8_t payloadLen);
    
    // Wildlife detection messaging
    bool sendWildlifeDetection(const WildlifeDetection& detection);
    bool sendMotionAlert(int16_t panAngle, int16_t tiltAngle, uint8_t confidence);
    bool requestImageFromCamera(uint8_t targetCameraId, uint32_t timestamp);
    
    // Status and heartbeat
    bool sendHeartbeat();
    bool sendStatusUpdate();
    bool requestStatusFromCamera(uint8_t targetCameraId);
    void updateHeartbeat();
    
    // Mesh networking
    bool joinNetwork();
    void updateKnownCameras(const CameraStatus& status);
    int getKnownCameraCount() const { return knownCameraCount; }
    CameraStatus* getKnownCamera(int index);
    bool isOnline(uint8_t cameraId) const;
    
    // Emergency communication
    bool sendEmergencyAlert(uint8_t alertType, const char* message);
    bool sendBatteryLowWarning();
    bool sendMaintenanceRequest();
    
    // Message processing
    void processIncomingMessages();
    bool processMessage(const LoRaMessage& message);
    
    // Queue management
    bool queueMessage(const LoRaMessage& message);
    bool getQueuedMessage(LoRaMessage& message);
    void clearMessageQueue();
    int getQueuedMessageCount() const { return queueCount; }
    
    // Network analysis
    void printNetworkStatus() const;
    void printKnownCameras() const;
    int getNetworkRSSI() const;
    bool isNetworkHealthy() const;
    
    // Configuration
    void setCameraId(uint8_t id) { cameraId = id; }
    uint8_t getCameraId() const { return cameraId; }
    void setTransmissionPower(int power);
    void setFrequency(long frequency);
    
    // Power management
    void enterSleepMode();
    void exitSleepMode();
    void setLowPowerMode(bool enable);
    
    // Callbacks for message events
    typedef void (*MessageCallback)(const LoRaMessage& message);
    typedef void (*WildlifeCallback)(const WildlifeDetection& detection);
    typedef void (*StatusCallback)(const CameraStatus& status);
    
    void setMessageCallback(MessageCallback callback);
    void setWildlifeCallback(WildlifeCallback callback);
    void setStatusCallback(StatusCallback callback);

private:
    // Internal helper functions
    uint16_t calculateChecksum(const LoRaMessage& message);
    bool validateMessage(const LoRaMessage& message);
    void createMessage(LoRaMessage& message, uint8_t type, uint8_t dest, 
                      const uint8_t* payload, uint8_t payloadLen);
    
    // Network management
    void updateNetworkTopology();
    bool forwardMessage(const LoRaMessage& message);
    uint8_t findBestRoute(uint8_t targetId);
    
    // Statistics
    unsigned long messagesSent;
    unsigned long messagesReceived;
    unsigned long messagesLost;
    int averageRSSI;
    
    // Callback functions
    MessageCallback messageCallback;
    WildlifeCallback wildlifeCallback;
    StatusCallback statusCallback;
    
    // Power management
    bool lowPowerMode;
    unsigned long lastActivity;
};

// Global LoRa communication instance
extern WildlifeLoRaComm loraComm;

#endif // WILDLIFE_LORA_COMM_H