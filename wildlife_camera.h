/*
  ESP32-AI-CAM Wildlife Trail Camera Extensions
  
  Additional features for comprehensive wildlife trail camera system:
  - Pan/tilt servo control with 270° coverage
  - Solar power monitoring and management
  - LoRa communication for inter-camera coordination
  - Advanced power management for extended operation
*/

#ifndef WILDLIFE_CAMERA_H
#define WILDLIFE_CAMERA_H

#include <Arduino.h>

// Pin definitions for wildlife camera extensions
#define SERVO_PAN_PIN 14      // Pan servo control pin
#define SERVO_TILT_PIN 15     // Tilt servo control pin
#define SOLAR_VOLTAGE_PIN 35  // ADC pin for solar panel voltage monitoring
#define BATTERY_VOLTAGE_PIN 34 // ADC pin for battery voltage monitoring
#define IR_LED_PIN 4          // IR illumination control (using existing flash pin)
#define LORA_CS_PIN 5         // LoRa chip select
#define LORA_RST_PIN 2        // LoRa reset pin
#define LORA_DIO0_PIN 26      // LoRa interrupt pin

// Camera movement limits and speeds
#define PAN_MIN_ANGLE 0
#define PAN_MAX_ANGLE 270
#define TILT_MIN_ANGLE 0  
#define TILT_MAX_ANGLE 180
#define SERVO_SPEED_DELAY 20  // milliseconds between servo steps

// Power management thresholds
#define BATTERY_LOW_VOLTAGE 3.4      // Volts - enter power saving mode
#define BATTERY_CRITICAL_VOLTAGE 3.2 // Volts - shutdown non-essential functions
#define SOLAR_MIN_VOLTAGE 4.0        // Minimum solar voltage for charging
#define POWER_SAMPLE_INTERVAL 60000  // Check power every 60 seconds

// Camera position structure
struct CameraPosition {
  int pan_angle;
  int tilt_angle;
  unsigned long timestamp;
};

// Power status structure
struct PowerStatus {
  float battery_voltage;
  float solar_voltage;
  bool is_charging;
  bool low_battery;
  bool critical_battery;
  int battery_percentage;
  unsigned long last_update;
};

// LoRa message types
enum LoRaMessageType {
  LORA_HEARTBEAT = 1,
  LORA_MOTION_DETECTED = 2,
  LORA_CAMERA_POSITION = 3,
  LORA_POWER_STATUS = 4,
  LORA_REMOTE_CONTROL = 5,
  LORA_WEATHER_DATA = 6
};

// LoRa message structure
struct LoRaMessage {
  uint8_t camera_id;
  LoRaMessageType msg_type;
  uint32_t timestamp;
  uint8_t data[32]; // Flexible data payload
  uint8_t checksum;
};

// Simple servo control class (replaces ESP32Servo to avoid dependency)
class SimpleServo {
private:
  int pin;
  int current_angle;
  bool attached;

public:
  SimpleServo();
  void attach(int pin);
  void detach();
  void write(int angle);
  int read();
};

class WildlifeCamera {
private:
  SimpleServo pan_servo;
  SimpleServo tilt_servo;
  CameraPosition current_position;
  CameraPosition home_position;
  PowerStatus power_status;
  bool ir_illumination_active;
  bool power_save_mode;
  unsigned long last_power_check;
  unsigned long last_heartbeat;
  uint8_t camera_id;
  
  // Private methods
  void updatePowerStatus();
  void checkPowerManagement();
  void enablePowerSaveMode();
  void disablePowerSaveMode();
  uint8_t calculateChecksum(const LoRaMessage& msg);

public:
  WildlifeCamera(uint8_t id = 1);
  
  // Initialization
  bool begin();
  void setHomePosition(int pan, int tilt);
  
  // Camera movement
  bool panTo(int angle);
  bool tiltTo(int angle);
  bool moveTo(int pan_angle, int tilt_angle);
  bool moveToHome();
  bool sweepArea(int pan_start, int pan_end, int tilt_angle, int step_size = 10);
  
  // Power management
  PowerStatus getPowerStatus();
  bool isLowBattery();
  bool isCriticalBattery();
  bool isSolarCharging();
  void enterDeepSleep(uint64_t sleep_time_us);
  void optimizePowerConsumption();
  
  // Illumination control
  void enableIRIllumination();
  void disableIRIllumination();
  void setIRBrightness(int brightness); // 0-255
  
  // LoRa communication
  bool initLoRa();
  bool sendMessage(LoRaMessageType type, const uint8_t* data, size_t len);
  bool receiveMessage(LoRaMessage& msg);
  void sendHeartbeat();
  void sendMotionAlert(const CameraPosition& detection_position);
  void sendPowerStatus();
  
  // Remote control
  bool processRemoteCommand(const LoRaMessage& msg);
  void handleRemotePanTilt(int pan, int tilt);
  
  // Scheduled operations
  void performScheduledTasks();
  void performPeriodicSweep();
  
  // Diagnostics
  void printStatus();
  String getStatusString();
};

// Global wildlife camera instance
extern WildlifeCamera wildlifeCamera;

// Utility functions
float readBatteryVoltage();
float readSolarVoltage();
int calculateBatteryPercentage(float voltage);
bool isNightTime(); // Based on light sensor
void optimizeForNightOperation();
void optimizeForDayOperation();

#endif // WILDLIFE_CAMERA_H