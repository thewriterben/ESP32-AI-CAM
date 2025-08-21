/*
  ESP32-AI-CAM Wildlife Trail Camera Extensions Implementation
  
  Implementation of pan/tilt control, solar power management, and LoRa communication
*/

#include "wildlife_camera.h"
#include "weather_filter.h"
#include <string.h>

// External sensor functions
extern float get_tsl_lux();

// Simple servo implementation using PWM
SimpleServo::SimpleServo() {
  pin = -1;
  current_angle = 90;
  attached = false;
}

void SimpleServo::attach(int p) {
  pin = p;
  pinMode(pin, OUTPUT);
  attached = true;
  // Configure PWM channel for servo control - use pin number as channel for simplicity
  int channel = pin % 16; // ESP32 has 16 PWM channels
  ledcSetup(channel, 50, 16); // 50Hz, 16-bit resolution
  ledcAttachPin(pin, channel);
  write(current_angle);
}

void SimpleServo::detach() {
  if (attached) {
    ledcDetachPin(pin);
    attached = false;
  }
}

void SimpleServo::write(int angle) {
  if (!attached) return;
  
  angle = constrain(angle, 0, 180);
  current_angle = angle;
  
  // Convert angle to PWM duty cycle
  // Servo PWM: 1ms = 0°, 1.5ms = 90°, 2ms = 180°
  int channel = pin % 16;
  int duty = map(angle, 0, 180, 3277, 6554); // 16-bit PWM values for 1-2ms
  ledcWrite(channel, duty);
}

int SimpleServo::read() {
  return current_angle;
}

// Global wildlife camera instance
WildlifeCamera wildlifeCamera;

WildlifeCamera::WildlifeCamera(uint8_t id) {
  camera_id = id;
  current_position = {135, 90, 0}; // Start centered
  home_position = {135, 90, 0};    // Default home position
  ir_illumination_active = false;
  power_save_mode = false;
  last_power_check = 0;
  last_heartbeat = 0;
  
  // Initialize power status
  power_status = {0, 0, false, false, false, 0, 0};
}

bool WildlifeCamera::begin() {
  Serial.println("Initializing Wildlife Camera System...");
  
  // Initialize servos
  pan_servo.attach(SERVO_PAN_PIN);
  tilt_servo.attach(SERVO_TILT_PIN);
  
  // Set initial position
  moveTo(home_position.pan_angle, home_position.tilt_angle);
  delay(1000); // Allow servos to reach position
  
  // Initialize power monitoring pins
  pinMode(SOLAR_VOLTAGE_PIN, INPUT);
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  pinMode(IR_LED_PIN, OUTPUT);
  digitalWrite(IR_LED_PIN, LOW);
  
  // Initial power status check
  updatePowerStatus();
  
  // Initialize LoRa (if available)
  bool lora_ok = initLoRa();
  if (lora_ok) {
    Serial.println("LoRa communication initialized");
  } else {
    Serial.println("LoRa communication not available");
  }
  
  Serial.println("Wildlife Camera System initialized");
  printStatus();
  
  return true;
}

void WildlifeCamera::setHomePosition(int pan, int tilt) {
  home_position.pan_angle = constrain(pan, PAN_MIN_ANGLE, PAN_MAX_ANGLE);
  home_position.tilt_angle = constrain(tilt, TILT_MIN_ANGLE, TILT_MAX_ANGLE);
  home_position.timestamp = millis();
}

bool WildlifeCamera::panTo(int angle) {
  angle = constrain(angle, PAN_MIN_ANGLE, PAN_MAX_ANGLE);
  
  // Smooth movement to prevent jarring
  int current_angle = current_position.pan_angle;
  int step = (angle > current_angle) ? 1 : -1;
  
  while (current_angle != angle) {
    current_angle += step;
    pan_servo.write(current_angle);
    delay(SERVO_SPEED_DELAY);
  }
  
  current_position.pan_angle = angle;
  current_position.timestamp = millis();
  
  Serial.printf("Pan moved to %d degrees\n", angle);
  return true;
}

bool WildlifeCamera::tiltTo(int angle) {
  angle = constrain(angle, TILT_MIN_ANGLE, TILT_MAX_ANGLE);
  
  // Smooth movement to prevent jarring
  int current_angle = current_position.tilt_angle;
  int step = (angle > current_angle) ? 1 : -1;
  
  while (current_angle != angle) {
    current_angle += step;
    tilt_servo.write(current_angle);
    delay(SERVO_SPEED_DELAY);
  }
  
  current_position.tilt_angle = angle;
  current_position.timestamp = millis();
  
  Serial.printf("Tilt moved to %d degrees\n", angle);
  return true;
}

bool WildlifeCamera::moveTo(int pan_angle, int tilt_angle) {
  Serial.printf("Moving camera to pan=%d, tilt=%d\n", pan_angle, tilt_angle);
  
  // Move both axes simultaneously for efficiency
  pan_angle = constrain(pan_angle, PAN_MIN_ANGLE, PAN_MAX_ANGLE);
  tilt_angle = constrain(tilt_angle, TILT_MIN_ANGLE, TILT_MAX_ANGLE);
  
  int pan_current = current_position.pan_angle;
  int tilt_current = current_position.tilt_angle;
  
  int pan_steps = abs(pan_angle - pan_current);
  int tilt_steps = abs(tilt_angle - tilt_current);
  int max_steps = max(pan_steps, tilt_steps);
  
  if (max_steps == 0) return true; // Already at position
  
  for (int i = 0; i <= max_steps; i++) {
    // Calculate intermediate positions
    int pan_pos = pan_current + (pan_angle - pan_current) * i / max_steps;
    int tilt_pos = tilt_current + (tilt_angle - tilt_current) * i / max_steps;
    
    pan_servo.write(pan_pos);
    tilt_servo.write(tilt_pos);
    delay(SERVO_SPEED_DELAY);
  }
  
  current_position.pan_angle = pan_angle;
  current_position.tilt_angle = tilt_angle;
  current_position.timestamp = millis();
  
  return true;
}

bool WildlifeCamera::moveToHome() {
  Serial.println("Returning to home position");
  return moveTo(home_position.pan_angle, home_position.tilt_angle);
}

bool WildlifeCamera::sweepArea(int pan_start, int pan_end, int tilt_angle, int step_size) {
  Serial.printf("Sweeping area: pan %d to %d, tilt %d\n", pan_start, pan_end, tilt_angle);
  
  // Constrain parameters
  pan_start = constrain(pan_start, PAN_MIN_ANGLE, PAN_MAX_ANGLE);
  pan_end = constrain(pan_end, PAN_MIN_ANGLE, PAN_MAX_ANGLE);
  tilt_angle = constrain(tilt_angle, TILT_MIN_ANGLE, TILT_MAX_ANGLE);
  step_size = max(1, step_size);
  
  // Set tilt position first
  tiltTo(tilt_angle);
  delay(500);
  
  // Sweep pan axis
  int direction = (pan_end > pan_start) ? step_size : -step_size;
  for (int pan = pan_start; 
       (direction > 0) ? (pan <= pan_end) : (pan >= pan_end); 
       pan += direction) {
    
    panTo(pan);
    delay(1000); // Pause at each position for detection
    
    // Check for motion during sweep
    // This could trigger image capture or analysis
  }
  
  return true;
}

void WildlifeCamera::updatePowerStatus() {
  power_status.battery_voltage = readBatteryVoltage();
  power_status.solar_voltage = readSolarVoltage();
  power_status.is_charging = (power_status.solar_voltage > SOLAR_MIN_VOLTAGE) && 
                            (power_status.solar_voltage > power_status.battery_voltage + 0.5);
  power_status.low_battery = power_status.battery_voltage < BATTERY_LOW_VOLTAGE;
  power_status.critical_battery = power_status.battery_voltage < BATTERY_CRITICAL_VOLTAGE;
  power_status.battery_percentage = calculateBatteryPercentage(power_status.battery_voltage);
  power_status.last_update = millis();
}

PowerStatus WildlifeCamera::getPowerStatus() {
  if (millis() - power_status.last_update > POWER_SAMPLE_INTERVAL) {
    updatePowerStatus();
  }
  return power_status;
}

bool WildlifeCamera::isLowBattery() {
  return getPowerStatus().low_battery;
}

bool WildlifeCamera::isCriticalBattery() {
  return getPowerStatus().critical_battery;
}

bool WildlifeCamera::isSolarCharging() {
  return getPowerStatus().is_charging;
}

void WildlifeCamera::checkPowerManagement() {
  updatePowerStatus();
  
  if (power_status.critical_battery && !power_save_mode) {
    Serial.println("Critical battery level - enabling power save mode");
    enablePowerSaveMode();
  } else if (power_status.low_battery && !power_save_mode) {
    Serial.println("Low battery level - optimizing power consumption");
    optimizePowerConsumption();
  } else if (!power_status.low_battery && power_save_mode) {
    Serial.println("Battery level recovered - disabling power save mode");
    disablePowerSaveMode();
  }
}

void WildlifeCamera::enablePowerSaveMode() {
  power_save_mode = true;
  
  // Disable non-essential functions
  disableIRIllumination();
  
  // Return to home position to save servo power
  moveToHome();
  
  // Detach servos to save power
  pan_servo.detach();
  tilt_servo.detach();
  
  Serial.println("Power save mode enabled");
}

void WildlifeCamera::disablePowerSaveMode() {
  power_save_mode = false;
  
  // Re-attach servos
  pan_servo.attach(SERVO_PAN_PIN);
  tilt_servo.attach(SERVO_TILT_PIN);
  
  // Return to last known position
  moveTo(current_position.pan_angle, current_position.tilt_angle);
  
  Serial.println("Power save mode disabled");
}

void WildlifeCamera::optimizePowerConsumption() {
  // Reduce servo update frequency
  // Dim IR illumination if active
  if (ir_illumination_active) {
    setIRBrightness(128); // 50% brightness
  }
}

void WildlifeCamera::enableIRIllumination() {
  if (isCriticalBattery()) {
    Serial.println("Cannot enable IR - critical battery");
    return;
  }
  
  digitalWrite(IR_LED_PIN, HIGH);
  ir_illumination_active = true;
  Serial.println("IR illumination enabled");
}

void WildlifeCamera::disableIRIllumination() {
  digitalWrite(IR_LED_PIN, LOW);
  ir_illumination_active = false;
  Serial.println("IR illumination disabled");
}

void WildlifeCamera::setIRBrightness(int brightness) {
  if (!ir_illumination_active) return;
  
  brightness = constrain(brightness, 0, 255);
  analogWrite(IR_LED_PIN, brightness);
  Serial.printf("IR brightness set to %d\n", brightness);
}

bool WildlifeCamera::initLoRa() {
  // LoRa initialization would go here
  // For now, return false to indicate not implemented
  Serial.println("LoRa initialization placeholder");
  return false;
}

bool WildlifeCamera::sendMessage(LoRaMessageType type, const uint8_t* data, size_t len) {
  // LoRa message sending would go here
  Serial.printf("LoRa message send placeholder: type=%d, len=%d\n", type, len);
  return false;
}

void WildlifeCamera::sendHeartbeat() {
  uint8_t heartbeat_data[8];
  memcpy(heartbeat_data, &power_status.battery_percentage, 1);
  memcpy(heartbeat_data + 1, &current_position.pan_angle, 2);
  memcpy(heartbeat_data + 3, &current_position.tilt_angle, 2);
  
  sendMessage(LORA_HEARTBEAT, heartbeat_data, 5);
  last_heartbeat = millis();
}

void WildlifeCamera::sendMotionAlert(const CameraPosition& detection_position) {
  uint8_t motion_data[8];
  memcpy(motion_data, &detection_position.pan_angle, 2);
  memcpy(motion_data + 2, &detection_position.tilt_angle, 2);
  memcpy(motion_data + 4, &detection_position.timestamp, 4);
  
  sendMessage(LORA_MOTION_DETECTED, motion_data, 8);
}

void WildlifeCamera::performScheduledTasks() {
  unsigned long now = millis();
  
  // Check power status periodically
  if (now - last_power_check > POWER_SAMPLE_INTERVAL) {
    checkPowerManagement();
    last_power_check = now;
  }
  
  // Send heartbeat every 5 minutes
  if (now - last_heartbeat > 300000) {
    sendHeartbeat();
  }
  
  // Automatic night/day optimization
  if (isNightTime() && !ir_illumination_active) {
    optimizeForNightOperation();
  } else if (!isNightTime() && ir_illumination_active) {
    optimizeForDayOperation();
  }
}

void WildlifeCamera::performPeriodicSweep() {
  if (power_save_mode) return; // Skip sweep in power save mode
  
  Serial.println("Performing periodic area sweep");
  
  // Save current position
  CameraPosition saved_position = current_position;
  
  // Perform 180-degree sweep at current tilt
  sweepArea(PAN_MIN_ANGLE, PAN_MAX_ANGLE, current_position.tilt_angle, 30);
  
  // Return to original position
  moveTo(saved_position.pan_angle, saved_position.tilt_angle);
}

void WildlifeCamera::printStatus() {
  Serial.println("=== Wildlife Camera Status ===");
  Serial.printf("Camera ID: %d\n", camera_id);
  Serial.printf("Position: Pan=%d°, Tilt=%d°\n", current_position.pan_angle, current_position.tilt_angle);
  Serial.printf("Home: Pan=%d°, Tilt=%d°\n", home_position.pan_angle, home_position.tilt_angle);
  Serial.printf("Battery: %.2fV (%d%%)\n", power_status.battery_voltage, power_status.battery_percentage);
  Serial.printf("Solar: %.2fV %s\n", power_status.solar_voltage, power_status.is_charging ? "(CHARGING)" : "");
  Serial.printf("IR Illumination: %s\n", ir_illumination_active ? "ON" : "OFF");
  Serial.printf("Power Save Mode: %s\n", power_save_mode ? "ON" : "OFF");
  Serial.printf("Weather Filter: %s\n", weatherFilter.getFilterStatus().c_str());
}

String WildlifeCamera::getStatusString() {
  String status = "CAM" + String(camera_id) + ":";
  status += "P" + String(current_position.pan_angle) + ",";
  status += "T" + String(current_position.tilt_angle) + ",";
  status += "B" + String(power_status.battery_percentage) + "%,";
  status += power_status.is_charging ? "CHG," : "BAT,";
  status += power_save_mode ? "PWR," : "NRM,";
  status += ir_illumination_active ? "IR" : "VIS";
  return status;
}

// Utility functions
float readBatteryVoltage() {
  int adc_value = analogRead(BATTERY_VOLTAGE_PIN);
  // Convert ADC reading to voltage (assuming voltage divider)
  // Adjust multiplier based on actual voltage divider ratio
  float voltage = (adc_value * 3.3 / 4095.0) * 2.0; // Assuming 2:1 voltage divider
  return voltage;
}

float readSolarVoltage() {
  int adc_value = analogRead(SOLAR_VOLTAGE_PIN);
  // Convert ADC reading to voltage
  float voltage = (adc_value * 3.3 / 4095.0) * 3.0; // Assuming 3:1 voltage divider for higher solar voltage
  return voltage;
}

int calculateBatteryPercentage(float voltage) {
  // Li-ion battery voltage to percentage approximation
  if (voltage >= 4.1) return 100;
  if (voltage >= 3.9) return 80;
  if (voltage >= 3.7) return 60;
  if (voltage >= 3.5) return 40;
  if (voltage >= 3.3) return 20;
  if (voltage >= 3.0) return 10;
  return 0;
}

bool isNightTime() {
  float lux = get_tsl_lux();
  return lux < 10.0; // Consider it night if less than 10 lux
}

void optimizeForNightOperation() {
  Serial.println("Optimizing for night operation");
  wildlifeCamera.enableIRIllumination();
  // Could adjust camera sensitivity, reduce sweep frequency, etc.
}

void optimizeForDayOperation() {
  Serial.println("Optimizing for day operation");
  wildlifeCamera.disableIRIllumination();
  // Could increase sweep frequency, adjust camera settings, etc.
}