/*
  ESP32-AI-CAM Weather Motion Filtering System Implementation
  
  Advanced algorithms for filtering weather-related motion from wildlife detection
*/

#include "weather_filter.h"
#include "settings.h"
#include <math.h>
#include <string.h>

// External sensor functions (from esp32_ai_cam_sfsensors.cpp)
extern float get_bme_temperature();
extern float get_bme_pressure(); 
extern float get_bme_humidity();
extern float get_tsl_lux();

// Global weather filter instance
WeatherFilter weatherFilter;

WeatherFilter::WeatherFilter() {
  motion_index = 0;
  env_index = 0;
  base_sensitivity = 55.0; // Default ADXL345 threshold
  current_sensitivity = base_sensitivity;
  last_valid_motion = 0;
  
  // Initialize buffers
  for (int i = 0; i < MOTION_SAMPLE_WINDOW; i++) {
    motion_buffer[i] = {0, 0, 0, 0, 0};
  }
  for (int i = 0; i < 10; i++) {
    env_history[i] = {0, 0, 0, 0, 0, 0, 0, false, false, false, 0};
  }
}

bool WeatherFilter::isValidWildlifeMotion(float x, float y, float z, const EnvironmentalConditions& env) {
  // Add new motion sample
  addMotionSample(x, y, z);
  addEnvironmentalSample(env);
  
  // Quick rejection for obvious weather conditions
  if (isWeatherEvent()) {
    Serial.println("Weather event detected - motion filtered");
    return false;
  }
  
  // Classify the motion pattern
  MotionPattern pattern = classifyMotion();
  
  // Apply pattern-based filtering
  switch (pattern) {
    case PATTERN_WILDLIFE:
      Serial.println("Wildlife pattern detected");
      last_valid_motion = millis();
      return true;
      
    case PATTERN_WIND:
      Serial.println("Wind pattern detected - motion filtered");
      return false;
      
    case PATTERN_RAIN:
      Serial.println("Rain pattern detected - motion filtered");
      return false;
      
    case PATTERN_VEGETATION:
      Serial.println("Vegetation movement detected - motion filtered");
      return false;
      
    case PATTERN_WATER:
      Serial.println("Water movement detected - motion filtered");
      return false;
      
    default:
      // Unknown pattern - use environmental context
      if (env.is_raining || env.is_windy || env.is_stormy) {
        Serial.println("Unknown pattern during weather event - motion filtered");
        return false;
      }
      Serial.println("Unknown pattern in calm conditions - allowing trigger");
      return true;
  }
}

void WeatherFilter::addMotionSample(float x, float y, float z) {
  float magnitude = sqrt(x*x + y*y + z*z);
  motion_buffer[motion_index] = {x, y, z, magnitude, millis()};
  motion_index = (motion_index + 1) % MOTION_SAMPLE_WINDOW;
}

void WeatherFilter::addEnvironmentalSample(const EnvironmentalConditions& env) {
  env_history[env_index] = env;
  updateEnvironmentalTrends(env);
  env_index = (env_index + 1) % 10;
}

MotionPattern WeatherFilter::classifyMotion() {
  // Need sufficient samples for analysis
  if (motion_index < 10) {
    return PATTERN_UNKNOWN;
  }
  
  // Analyze motion characteristics
  bool is_rhythmic = detectRhythmicPattern();
  bool is_random = detectRandomPattern();
  bool is_water_drops = detectWaterDroplets();
  bool is_vegetation = detectVegetationSway();
  
  // Get current environmental conditions
  EnvironmentalConditions current_env = env_history[(env_index - 1 + 10) % 10];
  
  // Classification logic
  if (is_water_drops && (current_env.is_raining || current_env.humidity > HUMIDITY_RAIN_THRESHOLD)) {
    return PATTERN_RAIN;
  }
  
  if (is_rhythmic && current_env.is_windy) {
    return PATTERN_WIND;
  }
  
  if (is_vegetation && current_env.is_windy) {
    return PATTERN_VEGETATION;
  }
  
  if (is_water_drops && !current_env.is_raining) {
    return PATTERN_WATER;
  }
  
  if (is_random && !current_env.is_windy && !current_env.is_raining) {
    return PATTERN_WILDLIFE;
  }
  
  return PATTERN_UNKNOWN;
}

bool WeatherFilter::detectRhythmicPattern() {
  // Calculate correlation between motion samples to detect repetitive patterns
  float correlation = calculateMotionCorrelation();
  return correlation > WIND_PATTERN_THRESHOLD;
}

bool WeatherFilter::detectRandomPattern() {
  // Wildlife motion tends to be more random and directional
  float correlation = calculateMotionCorrelation();
  float frequency = calculateMotionFrequency();
  
  // Random motion has low correlation and variable frequency
  return (correlation < 0.3 && frequency < 2.0);
}

bool WeatherFilter::detectWaterDroplets() {
  // Water droplets create brief, high-magnitude spikes
  int spike_count = 0;
  float avg_magnitude = 0;
  
  for (int i = 0; i < MOTION_SAMPLE_WINDOW; i++) {
    avg_magnitude += motion_buffer[i].magnitude;
  }
  avg_magnitude /= MOTION_SAMPLE_WINDOW;
  
  for (int i = 0; i < MOTION_SAMPLE_WINDOW; i++) {
    if (motion_buffer[i].magnitude > avg_magnitude * 2.5) {
      spike_count++;
    }
  }
  
  return spike_count > MOTION_SAMPLE_WINDOW * 0.1; // More than 10% spikes
}

bool WeatherFilter::detectVegetationSway() {
  // Vegetation creates predominantly horizontal motion with consistent frequency
  float horizontal_ratio = 0;
  float vertical_ratio = 0;
  
  for (int i = 0; i < MOTION_SAMPLE_WINDOW; i++) {
    float horizontal = sqrt(motion_buffer[i].x * motion_buffer[i].x + motion_buffer[i].y * motion_buffer[i].y);
    horizontal_ratio += horizontal;
    vertical_ratio += abs(motion_buffer[i].z);
  }
  
  // Vegetation sway is primarily horizontal
  return (horizontal_ratio > vertical_ratio * 2.0) && detectRhythmicPattern();
}

float WeatherFilter::calculateMotionCorrelation() {
  // Calculate autocorrelation to detect repetitive patterns
  if (motion_index < 20) return 0;
  
  float correlation = 0;
  int samples = min(MOTION_SAMPLE_WINDOW / 2, motion_index);
  
  for (int lag = 1; lag < 10; lag++) {
    float sum = 0;
    int count = 0;
    
    for (int i = lag; i < samples; i++) {
      int idx1 = (motion_index - i) % MOTION_SAMPLE_WINDOW;
      int idx2 = (motion_index - i + lag) % MOTION_SAMPLE_WINDOW;
      sum += motion_buffer[idx1].magnitude * motion_buffer[idx2].magnitude;
      count++;
    }
    
    if (count > 0) {
      correlation = max(correlation, sum / count);
    }
  }
  
  return correlation / 1000.0; // Normalize
}

float WeatherFilter::calculateMotionFrequency() {
  // Estimate dominant frequency in motion pattern
  int zero_crossings = 0;
  float prev_value = motion_buffer[0].magnitude;
  
  for (int i = 1; i < min(MOTION_SAMPLE_WINDOW, motion_index); i++) {
    float current_value = motion_buffer[i].magnitude;
    if ((prev_value > 0 && current_value <= 0) || (prev_value <= 0 && current_value > 0)) {
      zero_crossings++;
    }
    prev_value = current_value;
  }
  
  // Approximate frequency in Hz (assuming ~100ms sampling)
  return zero_crossings / 2.0 / (MOTION_SAMPLE_WINDOW * 0.1);
}

void WeatherFilter::updateEnvironmentalTrends(const EnvironmentalConditions& current) {
  // Calculate trends from recent history
  if (env_index < 3) return; // Need some history
  
  float pressure_sum = 0, temp_sum = 0, humidity_sum = 0;
  int count = min(env_index, 5); // Use last 5 readings
  
  for (int i = 0; i < count; i++) {
    int idx = (env_index - i - 1 + 10) % 10;
    pressure_sum += env_history[idx].pressure;
    temp_sum += env_history[idx].temperature;
    humidity_sum += env_history[idx].humidity;
  }
  
  float avg_pressure = pressure_sum / count;
  float avg_temp = temp_sum / count;
  float avg_humidity = humidity_sum / count;
  
  // Update current sample with trends
  EnvironmentalConditions* current_ptr = const_cast<EnvironmentalConditions*>(&current);
  current_ptr->pressure_trend = current.pressure - avg_pressure;
  current_ptr->temp_trend = current.temperature - avg_temp;
  current_ptr->humidity_trend = current.humidity - avg_humidity;
  
  // Detect weather conditions
  current_ptr->is_raining = (current.humidity > HUMIDITY_RAIN_THRESHOLD) || 
                           (current.lux < RAIN_LIGHT_THRESHOLD && current.humidity > 70);
  current_ptr->is_windy = abs(current_ptr->pressure_trend) > PRESSURE_CHANGE_THRESHOLD;
  current_ptr->is_stormy = current_ptr->is_raining && current_ptr->is_windy;
}

bool WeatherFilter::isWeatherEvent() {
  if (env_index == 0) return false;
  
  EnvironmentalConditions current = env_history[(env_index - 1 + 10) % 10];
  return current.is_raining || current.is_windy || current.is_stormy;
}

bool WeatherFilter::shouldTriggerCamera() {
  adjustSensitivity();
  return true; // Let the main filtering logic decide
}

void WeatherFilter::adjustSensitivity() {
  float weather_factor = calculateWeatherFactor();
  current_sensitivity = base_sensitivity * weather_factor;
  
  // Clamp sensitivity values
  current_sensitivity = max(30.0f, min(100.0f, current_sensitivity));
}

float WeatherFilter::calculateWeatherFactor() {
  if (env_index == 0) return 1.0;
  
  EnvironmentalConditions current = env_history[(env_index - 1 + 10) % 10];
  float factor = 1.0;
  
  // Increase sensitivity (lower threshold) in calm conditions
  if (!current.is_raining && !current.is_windy) {
    factor *= 0.8; // 20% more sensitive
  }
  
  // Decrease sensitivity (higher threshold) in bad weather
  if (current.is_stormy) {
    factor *= 2.0; // 50% less sensitive
  } else if (current.is_raining || current.is_windy) {
    factor *= 1.5; // 33% less sensitive
  }
  
  return factor;
}

float WeatherFilter::getAdaptiveSensitivity() {
  return current_sensitivity;
}

void WeatherFilter::setBaseSensitivity(float sensitivity) {
  base_sensitivity = sensitivity;
}

void WeatherFilter::resetFilter() {
  motion_index = 0;
  env_index = 0;
  current_sensitivity = base_sensitivity;
}

void WeatherFilter::printDiagnostics() {
  Serial.println("=== Weather Filter Diagnostics ===");
  Serial.printf("Motion samples: %d\n", motion_index);
  Serial.printf("Environmental samples: %d\n", env_index);
  Serial.printf("Base sensitivity: %.1f\n", base_sensitivity);
  Serial.printf("Current sensitivity: %.1f\n", current_sensitivity);
  
  if (env_index > 0) {
    EnvironmentalConditions current = env_history[(env_index - 1 + 10) % 10];
    Serial.printf("Current conditions - Rain: %s, Windy: %s, Storm: %s\n", 
                  current.is_raining ? "Yes" : "No",
                  current.is_windy ? "Yes" : "No", 
                  current.is_stormy ? "Yes" : "No");
    Serial.printf("Humidity: %.1f%%, Pressure trend: %.1f hPa\n", 
                  current.humidity, current.pressure_trend);
  }
}

String WeatherFilter::getFilterStatus() {
  String status = "Weather Filter: ";
  if (env_index > 0) {
    EnvironmentalConditions current = env_history[(env_index - 1 + 10) % 10];
    if (current.is_stormy) status += "STORM";
    else if (current.is_raining) status += "RAIN";
    else if (current.is_windy) status += "WIND";
    else status += "CLEAR";
  } else {
    status += "INIT";
  }
  return status;
}

// Helper functions for sensor integration
EnvironmentalConditions getCurrentEnvironmentalConditions() {
  EnvironmentalConditions env;
  env.temperature = get_bme_temperature();
  env.humidity = get_bme_humidity();
  env.pressure = get_bme_pressure();
  env.lux = get_tsl_lux();
  env.timestamp = millis();
  
  // Trends and conditions will be calculated in updateEnvironmentalTrends
  env.pressure_trend = 0;
  env.temp_trend = 0;
  env.humidity_trend = 0;
  env.is_raining = false;
  env.is_windy = false;
  env.is_stormy = false;
  
  return env;
}

bool isMotionFiltered(float x, float y, float z) {
#if WEATHER_FILTERING_ENABLED
  EnvironmentalConditions env = getCurrentEnvironmentalConditions();
  return !weatherFilter.isValidWildlifeMotion(x, y, z, env);
#else
  return false; // No filtering if disabled
#endif
}