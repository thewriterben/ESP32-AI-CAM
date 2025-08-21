/*
  ESP32-AI-CAM Weather Motion Filtering System
  
  Comprehensive weather and water movement filtering to eliminate false triggers
  from rain, wind, snow, moving vegetation, and water reflections.
  
  Features:
  - Multi-sensor validation with BME280, TSL2561, ADXL345
  - Temporal pattern analysis for motion classification
  - Environmental condition awareness and adaptive sensitivity
  - Water movement and vegetation filtering algorithms
*/

#ifndef WEATHER_FILTER_H
#define WEATHER_FILTER_H

#include <Arduino.h>

// Weather filtering configuration
#define MOTION_SAMPLE_WINDOW 50        // Number of motion samples to analyze
#define PRESSURE_CHANGE_THRESHOLD 2.0  // hPa change indicating weather change
#define WIND_PATTERN_THRESHOLD 0.7     // Correlation threshold for rhythmic patterns
#define RAIN_LIGHT_THRESHOLD 0.3       // Light level change indicating rain
#define TEMP_CHANGE_THRESHOLD 2.0      // Temperature change indicating weather event
#define HUMIDITY_RAIN_THRESHOLD 85.0   // Humidity level indicating rain/fog

// Motion pattern types
enum MotionPattern {
  PATTERN_UNKNOWN = 0,
  PATTERN_WILDLIFE = 1,
  PATTERN_WIND = 2,
  PATTERN_RAIN = 3,
  PATTERN_VEGETATION = 4,
  PATTERN_WATER = 5
};

// Environmental conditions
struct EnvironmentalConditions {
  float temperature;
  float humidity; 
  float pressure;
  float lux;
  float pressure_trend;
  float temp_trend;
  float humidity_trend;
  bool is_raining;
  bool is_windy;
  bool is_stormy;
  unsigned long timestamp;
};

// Motion sample data
struct MotionSample {
  float x, y, z;
  float magnitude;
  unsigned long timestamp;
};

// Weather filtering class
class WeatherFilter {
private:
  MotionSample motion_buffer[MOTION_SAMPLE_WINDOW];
  EnvironmentalConditions env_history[10]; // Keep 10 readings for trend analysis
  int motion_index;
  int env_index;
  float base_sensitivity;
  float current_sensitivity;
  unsigned long last_valid_motion;
  
  // Pattern analysis functions
  float calculateMotionCorrelation();
  float calculateMotionFrequency();
  bool detectRhythmicPattern();
  bool detectRandomPattern();
  bool detectWaterDroplets();
  bool detectVegetationSway();
  
  // Environmental analysis
  void updateEnvironmentalTrends(const EnvironmentalConditions& current);
  bool isWeatherEvent();
  bool isRainCondition();
  bool isWindyCondition();
  bool isStormCondition();
  
  // Adaptive sensitivity
  void adjustSensitivity();
  float calculateWeatherFactor();

public:
  WeatherFilter();
  
  // Main filtering function
  bool isValidWildlifeMotion(float x, float y, float z, const EnvironmentalConditions& env);
  
  // Update functions
  void addMotionSample(float x, float y, float z);
  void addEnvironmentalSample(const EnvironmentalConditions& env);
  
  // Analysis functions
  MotionPattern classifyMotion();
  float getAdaptiveSensitivity();
  bool shouldTriggerCamera();
  
  // Configuration
  void setBaseSensitivity(float sensitivity);
  void resetFilter();
  
  // Diagnostics
  void printDiagnostics();
  String getFilterStatus();
};

// Global weather filter instance
extern WeatherFilter weatherFilter;

// Helper functions for sensor integration
EnvironmentalConditions getCurrentEnvironmentalConditions();
bool isMotionFiltered(float x, float y, float z);

#endif // WEATHER_FILTER_H