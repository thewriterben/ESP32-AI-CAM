/*
  Wildlife Trail Camera Power Management Module
  
  Manages solar charging, battery monitoring, and power conservation
  Optimized for extended operation with solar panel power
*/

#include "tcamera_plus_s3_config.h"
#include <Arduino.h>

#ifndef WILDLIFE_POWER_MANAGEMENT_H
#define WILDLIFE_POWER_MANAGEMENT_H

// Power states
enum PowerState {
    POWER_NORMAL = 0,
    POWER_CONSERVATION = 1,
    POWER_CRITICAL = 2,
    POWER_EMERGENCY = 3,
    POWER_CHARGING = 4
};

// Battery status
struct BatteryStatus {
    float voltage;           // Battery voltage in volts
    float current;          // Current draw in mA
    uint8_t percentage;     // Battery percentage (0-100)
    float temperature;      // Battery temperature in Celsius
    bool isCharging;        // Charging status
    uint32_t cycleCount;    // Charge/discharge cycles
    uint32_t timeRemaining; // Estimated time remaining in minutes
};

// Solar panel status
struct SolarStatus {
    float voltage;          // Solar panel voltage
    float current;          // Solar panel current
    float power;            // Solar panel power in watts
    bool isActive;          // Solar panel active status
    uint16_t dailyEnergy;   // Daily energy harvested in Wh
    uint8_t efficiency;     // Current efficiency percentage
};

// Power consumption tracking
struct PowerConsumption {
    float camera;           // Camera power consumption
    float servos;           // Servo power consumption
    float lora;             // LoRa power consumption
    float sensors;          // Sensor power consumption
    float system;           // System power consumption
    float total;            // Total power consumption
};

class WildlifePowerManagement {
private:
    bool powerInitialized;
    PowerState currentPowerState;
    BatteryStatus batteryStatus;
    SolarStatus solarStatus;
    PowerConsumption powerConsumption;
    
    // Power management parameters
    const float BATTERY_FULL_VOLTAGE = 4.2;
    const float BATTERY_EMPTY_VOLTAGE = 3.2;
    const float BATTERY_CRITICAL_VOLTAGE = 3.4;
    const float BATTERY_LOW_VOLTAGE = 3.6;
    
    const float SOLAR_MIN_VOLTAGE = 5.0;
    const float SOLAR_MAX_VOLTAGE = 8.0;
    
    // Power conservation settings
    const unsigned long CONSERVATION_CHECK_INTERVAL = 60000;    // 1 minute
    const unsigned long DEEP_SLEEP_DURATION = 3600000000ULL;    // 1 hour in microseconds
    const unsigned long EMERGENCY_SLEEP_DURATION = 14400000000ULL; // 4 hours
    
    // Measurement parameters
    const int ADC_RESOLUTION = 4096;
    const float ADC_VOLTAGE_REF = 3.3;
    const float BATTERY_VOLTAGE_DIVIDER = 2.0;  // Voltage divider ratio
    
    // Power consumption tracking
    unsigned long lastPowerCheck;
    unsigned long lastChargingTime;
    float dailyPowerConsumption;
    float dailyPowerGeneration;
    
    // Statistics
    uint32_t deepSleepCount;
    uint32_t lowBatteryEvents;
    unsigned long totalOperatingTime;

public:
    WildlifePowerManagement();
    ~WildlifePowerManagement();
    
    // Initialization and control
    bool initializePowerManagement();
    void deinitializePowerManagement();
    bool isInitialized() const { return powerInitialized; }
    
    // Battery monitoring
    bool updateBatteryStatus();
    BatteryStatus getBatteryStatus() const { return batteryStatus; }
    float getBatteryVoltage();
    uint8_t getBatteryPercentage();
    bool isBatteryLow() const;
    bool isBatteryCritical() const;
    uint32_t getEstimatedRuntime(); // in minutes
    
    // Solar panel monitoring
    bool updateSolarStatus();
    SolarStatus getSolarStatus() const { return solarStatus; }
    float getSolarVoltage();
    float getSolarPower();
    bool isSolarCharging() const;
    bool isSolarOptimal() const;
    
    // Power state management
    PowerState getCurrentPowerState() const { return currentPowerState; }
    void updatePowerState();
    bool enterConservationMode();
    bool enterCriticalMode();
    bool enterEmergencyMode();
    bool exitLowPowerMode();
    
    // Power consumption monitoring
    void updatePowerConsumption();
    PowerConsumption getPowerConsumption() const { return powerConsumption; }
    float getTotalPowerDraw();
    float getAveragePowerDraw();
    
    // Deep sleep management
    bool shouldEnterDeepSleep() const;
    void prepareForDeepSleep();
    void enterDeepSleep(uint64_t sleepTimeUs);
    void configureWakeupSources();
    
    // Charging control
    bool enableCharging();
    bool disableCharging();
    bool isChargingEnabled() const;
    void optimizeChargingRate();
    
    // Power distribution control
    bool enableCameraPower();
    bool disableCameraPower();
    bool enableServoPower();
    bool disableServoPower();
    bool enableLoRaPower();
    bool disableLoRaPower();
    bool enableSensorPower();
    bool disableSensorPower();
    
    // Power optimization
    void optimizePowerConsumption();
    void adjustCPUFrequency();
    void managePowerRails();
    bool enableSmartPowerManagement();
    
    // Energy harvesting optimization
    void optimizeSolarEfficiency();
    float calculateOptimalSolarAngle();
    void adjustSolarPanelAngle(); // For motorized solar tracking
    
    // Emergency power management
    bool handleLowBattery();
    bool handleCriticalBattery();
    void emergencyShutdown();
    void sendLowBatteryAlert();
    
    // Statistics and reporting
    void printPowerStatus() const;
    void printEnergyStatistics() const;
    void resetPowerStatistics();
    float getPowerEfficiency() const;
    
    // Configuration
    void setBatteryCapacity(uint16_t capacityMah);
    void setSolarPanelRating(float wattRating);
    void setPowerThresholds(float critical, float low, float full);
    void setConservationMode(bool aggressive);
    
    // Calibration
    bool calibrateBatteryMonitor();
    bool calibrateSolarMonitor();
    void saveCalibrationData();
    void loadCalibrationData();
    
    // Weather-based power management
    void setWeatherForecast(int sunnyHours, int cloudyHours);
    void adjustForWeatherConditions();
    
    // Callbacks for power events
    typedef void (*LowBatteryCallback)(float voltage);
    typedef void (*ChargingCallback)(bool isCharging);
    typedef void (*PowerStateCallback)(PowerState newState);
    
    void setLowBatteryCallback(LowBatteryCallback callback);
    void setChargingCallback(ChargingCallback callback);
    void setPowerStateCallback(PowerStateCallback callback);

private:
    // Internal helper functions
    float readBatteryVoltage();
    float readSolarVoltage();
    float readCurrent();
    float calculateBatteryPercentage(float voltage);
    uint32_t calculateTimeRemaining();
    
    // Power management algorithms
    void implementMPPT(); // Maximum Power Point Tracking
    void balancePowerBudget();
    void predictPowerRequirements();
    
    // Configuration storage
    uint16_t batteryCapacityMah;
    float solarPanelWatts;
    float criticalVoltageThreshold;
    float lowVoltageThreshold;
    bool aggressiveConservation;
    
    // Calibration data
    float batteryVoltageOffset;
    float batteryVoltageScale;
    float solarVoltageOffset;
    float solarVoltageScale;
    
    // Weather data
    int forecastSunnyHours;
    int forecastCloudyHours;
    
    // Callback functions
    LowBatteryCallback lowBatteryCallback;
    ChargingCallback chargingCallback;
    PowerStateCallback powerStateCallback;
};

// Global power management instance
extern WildlifePowerManagement powerManager;

#endif // WILDLIFE_POWER_MANAGEMENT_H