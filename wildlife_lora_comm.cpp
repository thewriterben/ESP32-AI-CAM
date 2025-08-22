/*
  Wildlife Trail Camera LoRa Communication Implementation
  
  Long-range communication between trail cameras with mesh networking
*/

#include "wildlife_lora_comm.h"
#include "tcamera_plus_s3_config.h"

// Global LoRa communication instance
WildlifeLoRaComm loraComm;

WildlifeLoRaComm::WildlifeLoRaComm(uint8_t camId) {
    cameraId = camId;
    loraInitialized = false;
    sequenceNumber = 0;
    lastHeartbeat = 0;
    lastMessageReceived = 0;
    knownCameraCount = 0;
    queueHead = 0;
    queueTail = 0;
    queueCount = 0;
    messagesSent = 0;
    messagesReceived = 0;
    messagesLost = 0;
    averageRSSI = 0;
    lowPowerMode = false;
    lastActivity = 0;
    messageCallback = nullptr;
    wildlifeCallback = nullptr;
    statusCallback = nullptr;
    
    // Initialize known cameras array
    for (int i = 0; i < MAX_KNOWN_CAMERAS; i++) {
        knownCameras[i] = {0};
    }
    
    // Initialize message queue
    for (int i = 0; i < MESSAGE_QUEUE_SIZE; i++) {
        messageQueue[i] = {0};
    }
}

WildlifeLoRaComm::~WildlifeLoRaComm() {
    deinitializeLoRa();
}

bool WildlifeLoRaComm::initializeLoRa() {
    if (loraInitialized) {
        return true;
    }
    
    Serial.println("Initializing LoRa communication...");
    
    // Configure LoRa pins
    LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
    
    // Initialize LoRa with specified frequency
    if (!LoRa.begin(LORA_FREQUENCY)) {
        Serial.println("LoRa initialization failed!");
        return false;
    }
    
    // Configure LoRa parameters
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.setTxPower(LORA_TX_POWER);
    
    // Enable CRC checking
    LoRa.enableCrc();
    
    loraInitialized = true;
    Serial.printf("LoRa initialized successfully for Camera ID: %d\n", cameraId);
    Serial.printf("Frequency: %.0f MHz\n", LORA_FREQUENCY / 1E6);
    Serial.printf("Bandwidth: %.0f kHz\n", LORA_BANDWIDTH / 1E3);
    Serial.printf("Spreading Factor: %d\n", LORA_SPREADING_FACTOR);
    
    // Send network join message
    joinNetwork();
    
    return true;
}

void WildlifeLoRaComm::deinitializeLoRa() {
    if (!loraInitialized) {
        return;
    }
    
    Serial.println("Deinitializing LoRa communication...");
    
    LoRa.end();
    loraInitialized = false;
    
    Serial.println("LoRa communication deinitialized");
}

bool WildlifeLoRaComm::sendMessage(const LoRaMessage& message) {
    if (!loraInitialized) {
        Serial.println("LoRa not initialized");
        return false;
    }
    
    // Validate message
    if (!validateMessage(message)) {
        Serial.println("Invalid message");
        return false;
    }
    
    // Begin packet
    LoRa.beginPacket();
    
    // Send message header
    LoRa.write(message.messageType);
    LoRa.write(message.sourceId);
    LoRa.write(message.destId);
    LoRa.write(message.sequenceNum);
    LoRa.write(message.payloadLength);
    
    // Send payload
    for (int i = 0; i < message.payloadLength; i++) {
        LoRa.write(message.payload[i]);
    }
    
    // Send checksum
    LoRa.write((uint8_t)(message.checksum >> 8));
    LoRa.write((uint8_t)(message.checksum & 0xFF));
    
    // End packet and send
    bool success = LoRa.endPacket();
    
    if (success) {
        messagesSent++;
        lastActivity = millis();
        Serial.printf("Message sent: Type=%02X, Dest=%d, Seq=%d\n", 
                     message.messageType, message.destId, message.sequenceNum);
    } else {
        Serial.println("Failed to send message");
    }
    
    return success;
}

bool WildlifeLoRaComm::receiveMessage(LoRaMessage& message) {
    if (!loraInitialized) {
        return false;
    }
    
    int packetSize = LoRa.parsePacket();
    if (packetSize == 0) {
        return false; // No packet received
    }
    
    // Check minimum packet size
    if (packetSize < 7) { // Header + checksum minimum
        Serial.println("Packet too small");
        return false;
    }
    
    // Read message header
    message.messageType = LoRa.read();
    message.sourceId = LoRa.read();
    message.destId = LoRa.read();
    message.sequenceNum = LoRa.read();
    message.payloadLength = LoRa.read();
    
    // Read payload
    for (int i = 0; i < message.payloadLength && i < 200; i++) {
        message.payload[i] = LoRa.read();
    }
    
    // Read checksum
    message.checksum = (LoRa.read() << 8) | LoRa.read();
    
    // Validate message
    if (!validateMessage(message)) {
        Serial.println("Invalid message received");
        return false;
    }
    
    messagesReceived++;
    lastMessageReceived = millis();
    averageRSSI = (averageRSSI * 9 + LoRa.packetRssi()) / 10; // Moving average
    
    Serial.printf("Message received: Type=%02X, Source=%d, RSSI=%d\n", 
                 message.messageType, message.sourceId, LoRa.packetRssi());
    
    return true;
}

bool WildlifeLoRaComm::sendBroadcast(uint8_t messageType, const uint8_t* payload, uint8_t payloadLen) {
    LoRaMessage message;
    createMessage(message, messageType, 0xFF, payload, payloadLen);
    return sendMessage(message);
}

bool WildlifeLoRaComm::sendWildlifeDetection(const WildlifeDetection& detection) {
    Serial.println("Sending wildlife detection alert");
    
    // Serialize wildlife detection data
    uint8_t payload[sizeof(WildlifeDetection)];
    memcpy(payload, &detection, sizeof(WildlifeDetection));
    
    bool success = sendBroadcast(LORA_MSG_WILDLIFE_DETECTED, payload, sizeof(WildlifeDetection));
    
    if (success && wildlifeCallback) {
        wildlifeCallback(detection);
    }
    
    return success;
}

bool WildlifeLoRaComm::sendMotionAlert(int16_t panAngle, int16_t tiltAngle, uint8_t confidence) {
    Serial.printf("Sending motion alert: Pan=%d°, Tilt=%d°, Confidence=%d%%\n", 
                  panAngle, tiltAngle, confidence);
    
    uint8_t payload[5];
    payload[0] = (uint8_t)(panAngle >> 8);
    payload[1] = (uint8_t)(panAngle & 0xFF);
    payload[2] = (uint8_t)(tiltAngle >> 8);
    payload[3] = (uint8_t)(tiltAngle & 0xFF);
    payload[4] = confidence;
    
    return sendBroadcast(LORA_MSG_MOTION_ALERT, payload, 5);
}

bool WildlifeLoRaComm::sendHeartbeat() {
    CameraStatus status;
    status.cameraId = cameraId;
    status.timestamp = millis();
    // Fill other status fields...
    
    uint8_t payload[sizeof(CameraStatus)];
    memcpy(payload, &status, sizeof(CameraStatus));
    
    bool success = sendBroadcast(LORA_MSG_HEARTBEAT, payload, sizeof(CameraStatus));
    
    if (success) {
        lastHeartbeat = millis();
        Serial.println("Heartbeat sent");
    }
    
    return success;
}

bool WildlifeLoRaComm::sendStatusUpdate() {
    CameraStatus status;
    status.cameraId = cameraId;
    status.timestamp = millis();
    // TODO: Fill with actual sensor data
    status.batteryVoltage = 3.7; // Placeholder
    status.temperature = 25;     // Placeholder
    status.rssi = averageRSSI;
    
    uint8_t payload[sizeof(CameraStatus)];
    memcpy(payload, &status, sizeof(CameraStatus));
    
    return sendBroadcast(LORA_MSG_STATUS_RESPONSE, payload, sizeof(CameraStatus));
}

bool WildlifeLoRaComm::joinNetwork() {
    Serial.println("Joining LoRa mesh network...");
    
    uint8_t payload[1] = {cameraId};
    return sendBroadcast(LORA_MSG_NETWORK_JOIN, payload, 1);
}

void WildlifeLoRaComm::updateKnownCameras(const CameraStatus& status) {
    // Find existing camera or add new one
    for (int i = 0; i < knownCameraCount; i++) {
        if (knownCameras[i].cameraId == status.cameraId) {
            knownCameras[i] = status;
            return;
        }
    }
    
    // Add new camera if space available
    if (knownCameraCount < MAX_KNOWN_CAMERAS) {
        knownCameras[knownCameraCount] = status;
        knownCameraCount++;
        Serial.printf("New camera added to network: ID=%d\n", status.cameraId);
    }
}

void WildlifeLoRaComm::processIncomingMessages() {
    LoRaMessage message;
    while (receiveMessage(message)) {
        processMessage(message);
    }
}

bool WildlifeLoRaComm::processMessage(const LoRaMessage& message) {
    // Skip messages from self
    if (message.sourceId == cameraId) {
        return false;
    }
    
    // Process based on message type
    switch (message.messageType) {
        case LORA_MSG_HEARTBEAT:
        case LORA_MSG_STATUS_RESPONSE: {
            if (message.payloadLength == sizeof(CameraStatus)) {
                CameraStatus status;
                memcpy(&status, message.payload, sizeof(CameraStatus));
                updateKnownCameras(status);
                
                if (statusCallback) {
                    statusCallback(status);
                }
            }
            break;
        }
        
        case LORA_MSG_WILDLIFE_DETECTED: {
            if (message.payloadLength == sizeof(WildlifeDetection)) {
                WildlifeDetection detection;
                memcpy(&detection, message.payload, sizeof(WildlifeDetection));
                Serial.printf("Wildlife detected by Camera %d: Species=%d, Confidence=%d%%\n", 
                             message.sourceId, detection.speciesType, detection.confidence);
                
                if (wildlifeCallback) {
                    wildlifeCallback(detection);
                }
            }
            break;
        }
        
        case LORA_MSG_MOTION_ALERT: {
            if (message.payloadLength == 5) {
                int16_t panAngle = (message.payload[0] << 8) | message.payload[1];
                int16_t tiltAngle = (message.payload[2] << 8) | message.payload[3];
                uint8_t confidence = message.payload[4];
                Serial.printf("Motion alert from Camera %d: Pan=%d°, Tilt=%d°, Confidence=%d%%\n", 
                             message.sourceId, panAngle, tiltAngle, confidence);
            }
            break;
        }
        
        case LORA_MSG_STATUS_REQUEST: {
            // Respond with our status
            sendStatusUpdate();
            break;
        }
        
        case LORA_MSG_NETWORK_JOIN: {
            Serial.printf("Camera %d joined the network\n", message.sourceId);
            // Send our status to the new camera
            sendStatusUpdate();
            break;
        }
        
        case LORA_MSG_EMERGENCY: {
            Serial.printf("EMERGENCY alert from Camera %d\n", message.sourceId);
            // Handle emergency
            break;
        }
        
        case LORA_MSG_BATTERY_LOW: {
            Serial.printf("Battery low warning from Camera %d\n", message.sourceId);
            break;
        }
    }
    
    // Call general message callback
    if (messageCallback) {
        messageCallback(message);
    }
    
    return true;
}

void WildlifeLoRaComm::updateHeartbeat() {
    if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
        sendHeartbeat();
    }
}

void WildlifeLoRaComm::printNetworkStatus() const {
    Serial.println("=== LoRa Network Status ===");
    Serial.printf("Camera ID: %d\n", cameraId);
    Serial.printf("Messages sent: %lu\n", messagesSent);
    Serial.printf("Messages received: %lu\n", messagesReceived);
    Serial.printf("Average RSSI: %d dBm\n", averageRSSI);
    Serial.printf("Known cameras: %d\n", knownCameraCount);
    Serial.printf("Network healthy: %s\n", isNetworkHealthy() ? "Yes" : "No");
}

void WildlifeLoRaComm::printKnownCameras() const {
    Serial.println("=== Known Cameras ===");
    for (int i = 0; i < knownCameraCount; i++) {
        const CameraStatus& cam = knownCameras[i];
        Serial.printf("Camera %d: Battery=%.1fV, Temp=%d°C, RSSI=%d dBm\n", 
                     cam.cameraId, cam.batteryVoltage / 10.0, cam.temperature, cam.rssi);
    }
}

bool WildlifeLoRaComm::isNetworkHealthy() const {
    return (knownCameraCount > 0 && averageRSSI > RSSI_THRESHOLD);
}

// Private helper functions

uint16_t WildlifeLoRaComm::calculateChecksum(const LoRaMessage& message) {
    uint16_t checksum = 0;
    checksum += message.messageType;
    checksum += message.sourceId;
    checksum += message.destId;
    checksum += message.sequenceNum;
    checksum += message.payloadLength;
    
    for (int i = 0; i < message.payloadLength; i++) {
        checksum += message.payload[i];
    }
    
    return checksum;
}

bool WildlifeLoRaComm::validateMessage(const LoRaMessage& message) {
    if (message.payloadLength > 200) {
        return false;
    }
    
    uint16_t calculatedChecksum = calculateChecksum(message);
    return (calculatedChecksum == message.checksum);
}

void WildlifeLoRaComm::createMessage(LoRaMessage& message, uint8_t type, uint8_t dest, 
                                   const uint8_t* payload, uint8_t payloadLen) {
    message.messageType = type;
    message.sourceId = cameraId;
    message.destId = dest;
    message.sequenceNum = sequenceNumber++;
    message.payloadLength = payloadLen;
    
    if (payload && payloadLen > 0) {
        memcpy(message.payload, payload, payloadLen);
    }
    
    message.checksum = calculateChecksum(message);
}

void WildlifeLoRaComm::setMessageCallback(MessageCallback callback) {
    messageCallback = callback;
}

void WildlifeLoRaComm::setWildlifeCallback(WildlifeCallback callback) {
    wildlifeCallback = callback;
}

void WildlifeLoRaComm::setStatusCallback(StatusCallback callback) {
    statusCallback = callback;
}