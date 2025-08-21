/*
  Wildlife Trail Camera Power Management Implementation
  
  Solar charging, battery monitoring, and power conservation
*/

#include "wildlife_power_management.h"
#include "tcamera_plus_s3_config.h"
#include <esp_sleep.h>
#include <esp_pm.h>
#include <esp_cpu.h>

// Global power management instance
WildlifePowerManagement powerManager;

WildlifePowerManagement::WildlifePowerManagement() {
    powerInitialized = false;
    currentPowerState = POWER_NORMAL;
    lastPowerCheck = 0;
    lastChargingTime = 0;
    dailyPowerConsumption = 0.0;
    dailyPowerGeneration = 0.0;
    deepSleepCount = 0;
    lowBatteryEvents = 0;
    totalOperatingTime = 0;
    
    // Default configuration
    batteryCapacityMah = 2500; // 18650 typical capacity
    solarPanelWatts = 2.0;     // 2W solar panel
    criticalVoltageThreshold = BATTERY_CRITICAL_VOLTAGE;
    lowVoltageThreshold = BATTERY_LOW_VOLTAGE;
    aggressiveConservation = false;
    
    // Default calibration values
    batteryVoltageOffset = 0.0;
    batteryVoltageScale = 1.0;
    solarVoltageOffset = 0.0;
    solarVoltageScale = 1.0;
    
    // Weather defaults
    forecastSunnyHours = 8;
    forecastCloudyHours = 4;
    
    // Callback initialization
    lowBatteryCallback = nullptr;
    chargingCallback = nullptr;
    powerStateCallback = nullptr;
    
    // Initialize status structures
    batteryStatus = {0};
    solarStatus = {0};
    powerConsumption = {0};
}

WildlifePowerManagement::~WildlifePowerManagement() {
    deinitializePowerManagement();
}

bool WildlifePowerManagement::initializePowerManagement() {
    if (powerInitialized) {
        return true;
    }
    
    Serial.println("Initializing power management system...");
    
    // Configure ADC for battery monitoring
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    
    // Configure power control pins
    pinMode(BATTERY_MONITOR_PIN, INPUT);
    pinMode(SOLAR_PANEL_ENABLE_PIN, OUTPUT);
    pinMode(POWER_SAVE_MODE_PIN, OUTPUT);
    
    // Enable solar charging initially
    digitalWrite(SOLAR_PANEL_ENABLE_PIN, HIGH);
    digitalWrite(POWER_SAVE_MODE_PIN, LOW);
    
    // Load calibration data
    loadCalibrationData();
    
    // Initial power status update
    updateBatteryStatus();
    updateSolarStatus();
    updatePowerState();
    
    powerInitialized = true;
    Serial.println("Power management system initialized successfully");
    
    return true;
}

void WildlifePowerManagement::deinitializePowerManagement() {
    if (!powerInitialized) {
        return;
    }
    
    Serial.println("Deinitializing power management...");
    
    // Save calibration data
    saveCalibrationData();
    
    // Disable all power rails
    disableCharging();
    
    powerInitialized = false;
    Serial.println("Power management deinitialized");
}

bool WildlifePowerManagement::updateBatteryStatus() {
    if (!powerInitialized) {
        return false;
    }
    
    // Read battery voltage
    batteryStatus.voltage = readBatteryVoltage();
    batteryStatus.percentage = calculateBatteryPercentage(batteryStatus.voltage);
    batteryStatus.timeRemaining = calculateTimeRemaining();
    
    // Determine charging status
    float solarVoltage = readSolarVoltage();
    batteryStatus.isCharging = (solarVoltage > batteryStatus.voltage + 0.5);
    
    // Read temperature (placeholder - would need actual sensor)
    batteryStatus.temperature = 25.0; // Placeholder
    
    return true;
}

float WildlifePowerManagement::getBatteryVoltage() {
    return readBatteryVoltage();
}

uint8_t WildlifePowerManagement::getBatteryPercentage() {
    updateBatteryStatus();
    return batteryStatus.percentage;
}

bool WildlifePowerManagement::isBatteryLow() const {
    return batteryStatus.voltage < lowVoltageThreshold;
}

bool WildlifePowerManagement::isBatteryCritical() const {
    return batteryStatus.voltage < criticalVoltageThreshold;
}

uint32_t WildlifePowerManagement::getEstimatedRuntime() {
    return calculateTimeRemaining();
}

bool WildlifePowerManagement::updateSolarStatus() {
    if (!powerInitialized) {
        return false;
    }
    
    solarStatus.voltage = readSolarVoltage();
    solarStatus.current = readCurrent(); // Placeholder implementation
    solarStatus.power = solarStatus.voltage * solarStatus.current / 1000.0; // Convert to watts
    solarStatus.isActive = solarStatus.voltage > SOLAR_MIN_VOLTAGE;
    
    // Calculate efficiency (simplified)
    float theoreticalPower = solarPanelWatts;
    solarStatus.efficiency = (solarStatus.power / theoreticalPower) * 100.0;
    if (solarStatus.efficiency > 100) solarStatus.efficiency = 100;
    
    return true;
}

float WildlifePowerManagement::getSolarVoltage() {
    return readSolarVoltage();
}

float WildlifePowerManagement::getSolarPower() {
    updateSolarStatus();
    return solarStatus.power;
}

bool WildlifePowerManagement::isSolarCharging() const {
    return solarStatus.isActive && batteryStatus.isCharging;
}

bool WildlifePowerManagement::isSolarOptimal() const {
    return solarStatus.efficiency > 70; // Above 70% efficiency
}

void WildlifePowerManagement::updatePowerState() {
    PowerState previousState = currentPowerState;
    
    if (isBatteryCritical()) {
        currentPowerState = POWER_CRITICAL;
    } else if (isBatteryLow()) {
        currentPowerState = POWER_CONSERVATION;
    } else if (isSolarCharging()) {
        currentPowerState = POWER_CHARGING;
    } else {
        currentPowerState = POWER_NORMAL;
    }
    
    // Handle state transitions
    if (currentPowerState != previousState) {
        Serial.printf("Power state changed: %d -> %d\n", previousState, currentPowerState);
        
        if (powerStateCallback) {
            powerStateCallback(currentPowerState);
        }
        
        // Implement state-specific actions
        switch (currentPowerState) {
            case POWER_CRITICAL:
                enterCriticalMode();
                break;
            case POWER_CONSERVATION:
                enterConservationMode();
                break;
            case POWER_NORMAL:
                exitLowPowerMode();
                break;
            default:
                break;
        }
    }
}

bool WildlifePowerManagement::enterConservationMode() {
    Serial.println("Entering power conservation mode");
    
    // Reduce CPU frequency
    esp_pm_config_esp32s3_t pm_config;
    pm_config.max_freq_mhz = 80;      // Reduce from 240MHz to 80MHz
    pm_config.min_freq_mhz = 10;      // Minimum frequency
    pm_config.light_sleep_enable = true;
    esp_pm_configure(&pm_config);
    
    // Disable non-essential peripherals
    disableServoPower();
    
    // Reduce LoRa transmission frequency
    // (Implementation would depend on LoRa module)
    
    return true;
}

bool WildlifePowerManagement::enterCriticalMode() {
    Serial.println("Entering critical power mode");
    
    // More aggressive power conservation
    esp_pm_config_esp32s3_t pm_config;
    pm_config.max_freq_mhz = 40;      // Very low frequency
    pm_config.min_freq_mhz = 10;
    pm_config.light_sleep_enable = true;
    esp_pm_configure(&pm_config);
    
    // Disable most peripherals
    disableServoPower();
    disableLoRaPower();
    disableSensorPower();
    
    // Send low battery alert
    sendLowBatteryAlert();
    
    return true;
}

bool WildlifePowerManagement::enterEmergencyMode() {
    Serial.println("Entering emergency power mode - preparing for deep sleep");
    
    // Disable all non-essential systems
    disableCameraPower();
    disableServoPower();
    disableLoRaPower();
    disableSensorPower();
    
    // Configure for emergency deep sleep
    prepareForDeepSleep();
    enterDeepSleep(EMERGENCY_SLEEP_DURATION);
    
    return true;
}

bool WildlifePowerManagement::exitLowPowerMode() {
    Serial.println("Exiting low power mode");
    
    // Restore normal CPU frequency
    esp_pm_config_esp32s3_t pm_config;
    pm_config.max_freq_mhz = 240;     // Full speed
    pm_config.min_freq_mhz = 40;
    pm_config.light_sleep_enable = false;
    esp_pm_configure(&pm_config);
    
    // Re-enable peripherals
    enableCameraPower();
    enableServoPower();
    enableLoRaPower();
    enableSensorPower();
    
    return true;
}

bool WildlifePowerManagement::shouldEnterDeepSleep() const {
    if (isBatteryCritical()) {
        return true;
    }
    
    // Check if it's nighttime and no motion for extended period
    // (This would integrate with PIR motion detection)
    
    return false;
}

void WildlifePowerManagement::prepareForDeepSleep() {
    Serial.println("Preparing for deep sleep...");
    
    // Save current state
    saveCalibrationData();
    
    // Disable all peripherals
    disableCameraPower();
    disableServoPower();
    disableLoRaPower();
    disableSensorPower();
    
    // Configure wake-up sources
    configureWakeupSources();
}

void WildlifePowerManagement::enterDeepSleep(uint64_t sleepTimeUs) {
    Serial.printf("Entering deep sleep for %llu microseconds\n", sleepTimeUs);
    
    prepareForDeepSleep();
    
    // Set timer wake-up
    esp_sleep_enable_timer_wakeup(sleepTimeUs);
    
    // Enter deep sleep
    esp_deep_sleep_start();
}

void WildlifePowerManagement::configureWakeupSources() {
    // Configure timer wakeup (default)
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_DURATION);
    
    // Configure PIR motion wakeup
    esp_sleep_enable_ext0_wakeup(PIR_MOTION_PIN, 1);
    
    // Configure low battery wakeup (if needed)
    // Additional wakeup sources can be added here
}

bool WildlifePowerManagement::enableCharging() {
    digitalWrite(SOLAR_PANEL_ENABLE_PIN, HIGH);
    Serial.println("Solar charging enabled");
    return true;
}

bool WildlifePowerManagement::disableCharging() {
    digitalWrite(SOLAR_PANEL_ENABLE_PIN, LOW);
    Serial.println("Solar charging disabled");
    return true;
}

bool WildlifePowerManagement::isChargingEnabled() const {
    return digitalRead(SOLAR_PANEL_ENABLE_PIN) == HIGH;
}

bool WildlifePowerManagement::enableCameraPower() {
    // Implementation depends on power switching circuit
    Serial.println("Camera power enabled");
    return true;
}

bool WildlifePowerManagement::disableCameraPower() {
    Serial.println("Camera power disabled");
    return true;
}

bool WildlifePowerManagement::enableServoPower() {
    Serial.println("Servo power enabled");
    return true;
}

bool WildlifePowerManagement::disableServoPower() {
    Serial.println("Servo power disabled");
    return true;
}

bool WildlifePowerManagement::enableLoRaPower() {
    Serial.println("LoRa power enabled");
    return true;
}

bool WildlifePowerManagement::disableLoRaPower() {
    Serial.println("LoRa power disabled");
    return true;
}

bool WildlifePowerManagement::enableSensorPower() {
    Serial.println("Sensor power enabled");
    return true;
}

bool WildlifePowerManagement::disableSensorPower() {
    Serial.println("Sensor power disabled");
    return true;
}

void WildlifePowerManagement::printPowerStatus() const {
    Serial.println("=== Power Management Status ===");
    Serial.printf("Power State: %d\n", currentPowerState);
    Serial.printf("Battery: %.2fV (%d%%)\n", batteryStatus.voltage, batteryStatus.percentage);
    Serial.printf("Solar: %.2fV (%.1fW)\n", solarStatus.voltage, solarStatus.power);
    Serial.printf("Charging: %s\n", batteryStatus.isCharging ? "Yes" : "No");
    Serial.printf("Runtime: %lu minutes\n", batteryStatus.timeRemaining);
    Serial.printf("Deep Sleep Count: %lu\n", deepSleepCount);
    Serial.printf("Low Battery Events: %lu\n", lowBatteryEvents);
}

void WildlifePowerManagement::printEnergyStatistics() const {
    Serial.println("=== Energy Statistics ===");
    Serial.printf("Daily Power Consumption: %.2f Wh\n", dailyPowerConsumption);
    Serial.printf("Daily Power Generation: %.2f Wh\n", dailyPowerGeneration);
    Serial.printf("Power Efficiency: %.1f%%\n", getPowerEfficiency());
    Serial.printf("Total Operating Time: %lu hours\n", totalOperatingTime / 3600000);
}

float WildlifePowerManagement::getPowerEfficiency() const {
    if (dailyPowerConsumption == 0) {
        return 0.0;
    }
    return (dailyPowerGeneration / dailyPowerConsumption) * 100.0;
}

// Private helper functions

float WildlifePowerManagement::readBatteryVoltage() {
    int adcValue = analogRead(BATTERY_MONITOR_PIN);
    float voltage = (adcValue * ADC_VOLTAGE_REF / ADC_RESOLUTION) * BATTERY_VOLTAGE_DIVIDER;
    
    // Apply calibration
    voltage = (voltage + batteryVoltageOffset) * batteryVoltageScale;
    
    return voltage;
}

float WildlifePowerManagement::readSolarVoltage() {
    // Placeholder implementation - would read from solar voltage divider
    // For now, return a simulated value based on time of day
    int hour = (millis() / 3600000) % 24; // Rough hour calculation
    if (hour >= 6 && hour <= 18) {
        return 6.0 + (float)random(0, 100) / 100.0; // Daytime: 6.0-7.0V
    } else {
        return 0.5; // Nighttime: minimal voltage
    }
}

float WildlifePowerManagement::readCurrent() {
    // Placeholder implementation - would use current sensor
    return 50.0; // 50mA placeholder
}

float WildlifePowerManagement::calculateBatteryPercentage(float voltage) {
    if (voltage >= BATTERY_FULL_VOLTAGE) {
        return 100;
    } else if (voltage <= BATTERY_EMPTY_VOLTAGE) {
        return 0;
    } else {
        return ((voltage - BATTERY_EMPTY_VOLTAGE) / (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE)) * 100.0;
    }
}

uint32_t WildlifePowerManagement::calculateTimeRemaining() {
    // Simplified calculation based on current consumption
    float currentCapacity = (batteryStatus.percentage / 100.0) * batteryCapacityMah;
    float averageCurrentDraw = 100.0; // 100mA average (placeholder)
    
    return (uint32_t)((currentCapacity / averageCurrentDraw) * 60.0); // Minutes
}

void WildlifePowerManagement::sendLowBatteryAlert() {
    if (lowBatteryCallback) {
        lowBatteryCallback(batteryStatus.voltage);
    }
    
    lowBatteryEvents++;
    Serial.printf("Low battery alert sent: %.2fV\n", batteryStatus.voltage);
}

void WildlifePowerManagement::saveCalibrationData() {
    // Implementation would save to flash/EEPROM
    Serial.println("Calibration data saved");
}

void WildlifePowerManagement::loadCalibrationData() {
    // Implementation would load from flash/EEPROM
    Serial.println("Calibration data loaded");
}

void WildlifePowerManagement::setLowBatteryCallback(LowBatteryCallback callback) {
    lowBatteryCallback = callback;
}

void WildlifePowerManagement::setChargingCallback(ChargingCallback callback) {
    chargingCallback = callback;
}

void WildlifePowerManagement::setPowerStateCallback(PowerStateCallback callback) {
    powerStateCallback = callback;
}