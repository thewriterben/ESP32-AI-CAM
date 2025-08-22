/*
  Wildlife Trail Camera System - Main Program
  
  LilyGO T-Camera Plus S3 based wildlife monitoring system
  Integrates solar power, pan/tilt servos, PIR motion, LoRa communication,
  and AI-powered wildlife detection
  
  Based on original ESP32-AI-CAM by James Zahary
  Enhanced for wildlife trail camera application
*/

// Include new wildlife modules
#include "tcamera_plus_s3_config.h"
#include "wildlife_servo_control.h"
#include "wildlife_pir_motion.h"
#include "wildlife_lora_comm.h"
#include "wildlife_power_management.h"
#include "wildlife_ai_detection.h"

// Include original modules (updated for T-Camera Plus S3)
#include "settings.h"
#include <AzureIoTHub.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include "esp_camera.h"
#include "esp_sleep.h"
#include <ESP32Servo.h>
#include <LoRa.h>

// Camera and image handling
camera_fb_t *fb = NULL;
static uint8_t *framebuffer[10];
static int framebuffer_len[10];
static time_t framebuffer_now[10];
int num_pics = 3; // Number of pictures per motion event

// Environmental sensor data
float temperature[10];
float humidity[10];
float pressure[10];
float lux[10];

// System state
RTC_DATA_ATTR int bootCount = 0;
esp_sleep_wakeup_cause_t wakeup_reason;
bool systemInitialized = false;

// Function declarations
void setup();
void loop();
bool initializeWildlifeSystem();
void deinitializeWildlifeSystem();
bool initializeCamera();
void deinitializeCamera();
void handleWildlifeDetection();
void processMotionEvent();
void performWildlifeScan();
void handleLowBattery();
void enterDeepSleep();
void printSystemStatus();

// Callback functions
void onWildlifeDetected(const WildlifeDetection& detection);
void onMotionDetected(MotionEvent event);
void onLowBattery(float voltage);
void onLoRaMessage(const LoRaMessage& message);

void setup() {
    Serial.begin(115200);
    Serial.printf("\n\n=== Wildlife Trail Camera System v2.0 ===\n");
    Serial.printf("T-Camera Plus S3 with Solar Power Integration\n\n");
    
    delay(1000);
    
    // Check wakeup reason
    wakeup_reason = esp_sleep_get_wakeup_cause();
    ++bootCount;
    Serial.printf("Boot #%d, Wakeup cause: ", bootCount);
    
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0:
            Serial.println("PIR Motion Sensor");
            break;
        case ESP_SLEEP_WAKEUP_TIMER:
            Serial.println("Timer");
            break;
        case ESP_SLEEP_WAKEUP_TOUCHPAD:
            Serial.println("Touchpad");
            break;
        case ESP_SLEEP_WAKEUP_ULP:
            Serial.println("ULP program");
            break;
        default:
            Serial.printf("Not caused by deep sleep: %d\n", wakeup_reason);
            break;
    }
    
    // Initialize wildlife camera system
    if (!initializeWildlifeSystem()) {
        Serial.println("ERROR: Failed to initialize wildlife system!");
        Serial.println("Entering emergency deep sleep...");
        esp_deep_sleep_start();
        return;
    }
    
    systemInitialized = true;
    Serial.println("Wildlife camera system initialized successfully!");
    
    // Handle different wakeup scenarios
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0:
            // PIR motion detected
            Serial.println("Processing motion-triggered wakeup...");
            processMotionEvent();
            break;
            
        case ESP_SLEEP_WAKEUP_TIMER:
            // Scheduled wakeup
            Serial.println("Processing scheduled wakeup...");
            performWildlifeScan();
            break;
            
        default:
            // First boot or other wakeup
            Serial.println("Performing initial system check...");
            performWildlifeScan();
            break;
    }
    
    // Check power status and determine next action
    powerManager.updateBatteryStatus();
    powerManager.updatePowerState();
    
    if (powerManager.shouldEnterDeepSleep()) {
        Serial.println("Low battery detected - entering deep sleep");
        enterDeepSleep();
    } else {
        Serial.println("Entering main operation loop");
    }
}

void loop() {
    static unsigned long lastUpdate = 0;
    static unsigned long lastHeartbeat = 0;
    unsigned long currentTime = millis();
    
    // Update system components every second
    if (currentTime - lastUpdate > 1000) {
        lastUpdate = currentTime;
        
        // Update power management
        powerManager.updateBatteryStatus();
        powerManager.updateSolarStatus();
        powerManager.updatePowerState();
        
        // Check for motion
        if (pirMotion.checkMotion()) {
            Serial.println("Motion detected in main loop!");
            handleWildlifeDetection();
        }
        
        // Process incoming LoRa messages
        loraComm.processIncomingMessages();
        
        // Check for low battery
        if (powerManager.isBatteryCritical()) {
            handleLowBattery();
            return;
        }
    }
    
    // Send heartbeat every 5 minutes
    if (currentTime - lastHeartbeat > 300000) {
        lastHeartbeat = currentTime;
        loraComm.updateHeartbeat();
        
        // Print system status
        printSystemStatus();
    }
    
    // Check if we should enter deep sleep
    if (powerManager.shouldEnterDeepSleep()) {
        enterDeepSleep();
    }
    
    // Small delay to prevent watchdog issues
    delay(100);
}

bool initializeWildlifeSystem() {
    Serial.println("Initializing wildlife camera modules...");
    
    // Initialize power management first
    if (!powerManager.initializePowerManagement()) {
        Serial.println("Failed to initialize power management");
        return false;
    }
    
    // Check battery level before continuing
    if (powerManager.isBatteryCritical()) {
        Serial.println("Critical battery level - limited initialization");
        return true; // Continue with limited functionality
    }
    
    // Initialize PIR motion detection
    if (!pirMotion.initializePIR()) {
        Serial.println("Failed to initialize PIR motion detection");
        return false;
    }
    
    // Set up motion detection callback
    pirMotion.setMotionCallback(onMotionDetected);
    
    // Initialize servo control
    if (!servoControl.initializeServos()) {
        Serial.println("Failed to initialize servo control");
        return false;
    }
    
    // Move to home position
    servoControl.moveToHomePosition();
    delay(1000);
    
    // Initialize LoRa communication
    if (!loraComm.initializeLoRa()) {
        Serial.println("Failed to initialize LoRa communication");
        return false;
    }
    
    // Set up LoRa callbacks
    loraComm.setMessageCallback(onLoRaMessage);
    
    // Initialize AI detection
    if (!aiDetection.initializeAI()) {
        Serial.println("Failed to initialize AI detection");
        return false;
    }
    
    // Set up AI callbacks
    aiDetection.setWildlifeDetectionCallback(onWildlifeDetected);
    
    // Initialize camera
    if (!initializeCamera()) {
        Serial.println("Failed to initialize camera");
        return false;
    }
    
    // Set up power management callbacks
    powerManager.setLowBatteryCallback(onLowBattery);
    
    Serial.println("All wildlife camera modules initialized successfully");
    return true;
}

void deinitializeWildlifeSystem() {
    Serial.println("Deinitializing wildlife camera system...");
    
    deinitializeCamera();
    aiDetection.deinitializeAI();
    loraComm.deinitializeLoRa();
    servoControl.deinitializeServos();
    pirMotion.deinitializePIR();
    powerManager.deinitializePowerManagement();
    
    systemInitialized = false;
    Serial.println("Wildlife camera system deinitialized");
}

bool initializeCamera() {
    Serial.println("Initializing T-Camera Plus S3...");
    
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    
    if (psramFound()) {
        config.frame_size = CAMERA_RESOLUTION;
        config.jpeg_quality = CAMERA_JPEG_QUALITY;
        config.fb_count = 2;
        Serial.println("PSRAM found - using high quality settings");
    } else {
        config.frame_size = FRAMESIZE_VGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
        Serial.println("No PSRAM - using reduced quality settings");
    }
    
    // Initialize camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }
    
    // Get camera sensor
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        // Configure sensor for wildlife photography
        s->set_brightness(s, 0);     // -2 to 2
        s->set_contrast(s, 0);       // -2 to 2
        s->set_saturation(s, 0);     // -2 to 2
        s->set_special_effect(s, 0); // 0 to 6 (0-No Effect, 1-Negative, 2-Grayscale, 3-Red Tint, 4-Green Tint, 5-Blue Tint, 6-Sepia)
        s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
        s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
        s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
        s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
        s->set_aec2(s, 0);           // 0 = disable , 1 = enable
        s->set_ae_level(s, 0);       // -2 to 2
        s->set_aec_value(s, 300);    // 0 to 1200
        s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
        s->set_agc_gain(s, 0);       // 0 to 30
        s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
        s->set_bpc(s, 0);            // 0 = disable , 1 = enable
        s->set_wpc(s, 1);            // 0 = disable , 1 = enable
        s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
        s->set_lenc(s, 1);           // 0 = disable , 1 = enable
        s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
        s->set_vflip(s, 0);          // 0 = disable , 1 = enable
        s->set_dcw(s, 1);            // 0 = disable , 1 = enable
        s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
    }
    
    Serial.println("T-Camera Plus S3 initialized successfully");
    return true;
}

void deinitializeCamera() {
    esp_camera_deinit();
    Serial.println("Camera deinitialized");
}

void handleWildlifeDetection() {
    Serial.println("=== Wildlife Detection Event ===");
    
    if (!systemInitialized) {
        Serial.println("System not initialized - skipping detection");
        return;
    }
    
    // Capture images for analysis
    for (int i = 0; i < num_pics; i++) {
        Serial.printf("Capturing image %d/%d...\n", i + 1, num_pics);
        
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            continue;
        }
        
        // Analyze image with AI
        WildlifeDetection detection;
        if (aiDetection.analyzeFrame(fb->buf, fb->len, detection)) {
            Serial.printf("Wildlife detected: Species=%d, Confidence=%d%%\n", 
                         detection.species, detection.confidencePercent);
            
            // Track with servos if wildlife detected
            if (detection.species != SPECIES_UNKNOWN) {
                aiDetection.trackWithServos(detection);
                
                // Send LoRa alert to other cameras
                loraComm.sendWildlifeDetection({
                    .timestamp = detection.timestamp,
                    .speciesType = (uint8_t)detection.species,
                    .confidence = detection.confidencePercent,
                    .panAngle = (int16_t)servoControl.getCurrentPanAngle(),
                    .tiltAngle = (int16_t)servoControl.getCurrentTiltAngle(),
                    .batteryVoltage = powerManager.getBatteryVoltage()
                });
            }
        }
        
        // Store image (simplified - would save to SD card)
        Serial.printf("Image %d: %d bytes\n", i, fb->len);
        
        esp_camera_fb_return(fb);
        
        if (i < num_pics - 1) {
            delay(1000); // Wait between captures
        }
    }
    
    Serial.println("Wildlife detection event completed");
}

void processMotionEvent() {
    Serial.println("Processing PIR motion event...");
    
    // Check if motion is still active
    if (pirMotion.checkMotion() && pirMotion.shouldTriggerRecording()) {
        Serial.println("Valid wildlife motion detected");
        handleWildlifeDetection();
    } else {
        Serial.println("Motion event filtered out (likely false positive)");
    }
}

void performWildlifeScan() {
    Serial.println("Performing wildlife area scan...");
    
    if (!systemInitialized) {
        Serial.println("System not initialized - skipping scan");
        return;
    }
    
    // Perform automated scan pattern
    if (servoControl.performWildlifeScan()) {
        Serial.println("Wildlife scan completed");
    } else {
        Serial.println("Wildlife scan failed");
    }
    
    // Return to home position
    servoControl.moveToHomePosition();
}

void handleLowBattery() {
    Serial.println("=== LOW BATTERY DETECTED ===");
    
    // Send emergency LoRa message
    loraComm.sendBatteryLowWarning();
    
    // Disable non-essential systems
    powerManager.enterCriticalMode();
    
    // Enter emergency deep sleep
    enterDeepSleep();
}

void enterDeepSleep() {
    Serial.println("Preparing for deep sleep...");
    
    // Deinitialize non-essential systems
    if (systemInitialized) {
        aiDetection.deinitializeAI();
        servoControl.deinitializeServos();
        loraComm.deinitializeLoRa();
    }
    
    // Prepare for deep sleep
    powerManager.prepareForDeepSleep();
    
    // Configure wakeup sources
    esp_sleep_enable_ext0_wakeup(PIR_MOTION_PIN, 1);  // PIR motion
    esp_sleep_enable_timer_wakeup(3600ULL * 1000000); // 1 hour timer
    
    Serial.println("Entering deep sleep...");
    delay(100);
    
    esp_deep_sleep_start();
}

void printSystemStatus() {
    Serial.println("\n=== Wildlife Camera System Status ===");
    
    // Power status
    powerManager.printPowerStatus();
    
    // Motion detection stats
    pirMotion.printMotionStatistics();
    
    // AI detection stats
    aiDetection.printDetectionStatistics();
    
    // LoRa network status
    loraComm.printNetworkStatus();
    
    // Servo position
    servoControl.printCurrentPosition();
    
    Serial.println("=======================================\n");
}

// Callback functions
void onWildlifeDetected(const WildlifeDetection& detection) {
    Serial.printf("AI Callback: Wildlife detected - Species: %d, Confidence: %d%%\n", 
                  detection.species, detection.confidencePercent);
}

void onMotionDetected(MotionEvent event) {
    Serial.printf("PIR Callback: Motion detected - Confidence: %d\n", event.confidence);
}

void onLowBattery(float voltage) {
    Serial.printf("Power Callback: Low battery warning - %.2fV\n", voltage);
}

void onLoRaMessage(const LoRaMessage& message) {
    Serial.printf("LoRa Callback: Message received - Type: 0x%02X, Source: %d\n", 
                  message.messageType, message.sourceId);
}