/*
  ESP32-AI-CAM Wildlife Camera System - Test and Validation
  
  Test functions to validate weather filtering and camera control functionality
*/

#include "weather_filter.h"
#include "wildlife_camera.h"
#include "settings.h"
#include "system_tests.h"

// Test data for various motion patterns
struct TestMotionData {
  float x, y, z;
  const char* description;
  bool expected_valid; // true if should pass filter, false if should be filtered
};

// Wildlife motion patterns (should pass filter)
TestMotionData wildlife_patterns[] = {
  {10.5, -8.2, 15.3, "Deer walking", true},
  {-15.8, 12.1, -6.7, "Bird landing", true},
  {25.3, -18.9, 22.1, "Rabbit hopping", true},
  {8.7, 11.2, -9.4, "Squirrel movement", true}
};

// Weather motion patterns (should be filtered)
TestMotionData weather_patterns[] = {
  {2.1, 2.0, 1.9, "Light wind in branches", false},
  {0.8, 0.9, 0.7, "Rain droplets", false},
  {3.2, 3.1, 3.3, "Heavy wind sway", false},
  {0.5, 0.4, 0.6, "Water dripping", false}
};

// Environmental test conditions
EnvironmentalConditions test_conditions[] = {
  {22.5, 45.0, 1013.2, 500.0, 0, 0, 0, false, false, false, 0}, // Clear weather
  {18.3, 88.0, 1005.1, 50.0, -2.1, -1.2, 15.0, true, false, false, 0}, // Rainy
  {25.1, 35.0, 995.8, 800.0, -8.5, 2.1, -5.2, false, true, false, 0}, // Windy
  {15.2, 92.0, 988.3, 25.0, -12.3, -3.8, 25.8, true, true, true, 0}  // Storm
};

#if WEATHER_FILTERING_ENABLED

void testWeatherFiltering() {
  Serial.println("\n=== Weather Filtering Test Suite ===");
  
  int total_tests = 0;
  int passed_tests = 0;
  
  // Test wildlife patterns in clear weather
  Serial.println("\nTesting wildlife patterns in clear weather:");
  EnvironmentalConditions clear_weather = test_conditions[0];
  
  for (int i = 0; i < 4; i++) {
    TestMotionData pattern = wildlife_patterns[i];
    bool result = weatherFilter.isValidWildlifeMotion(pattern.x, pattern.y, pattern.z, clear_weather);
    
    total_tests++;
    if (result == pattern.expected_valid) {
      passed_tests++;
      Serial.printf("✓ %s: PASS (detected as %s)\n", 
                    pattern.description, result ? "wildlife" : "filtered");
    } else {
      Serial.printf("✗ %s: FAIL (expected %s, got %s)\n", 
                    pattern.description, 
                    pattern.expected_valid ? "wildlife" : "filtered",
                    result ? "wildlife" : "filtered");
    }
  }
  
  // Test weather patterns in various conditions
  Serial.println("\nTesting weather patterns in storm conditions:");
  EnvironmentalConditions storm_weather = test_conditions[3];
  
  for (int i = 0; i < 4; i++) {
    TestMotionData pattern = weather_patterns[i];
    bool result = weatherFilter.isValidWildlifeMotion(pattern.x, pattern.y, pattern.z, storm_weather);
    
    total_tests++;
    if (result == pattern.expected_valid) {
      passed_tests++;
      Serial.printf("✓ %s: PASS (correctly %s)\n", 
                    pattern.description, result ? "allowed" : "filtered");
    } else {
      Serial.printf("✗ %s: FAIL (expected %s, got %s)\n", 
                    pattern.description, 
                    pattern.expected_valid ? "allowed" : "filtered",
                    result ? "allowed" : "filtered");
    }
  }
  
  Serial.printf("\nWeather Filter Test Results: %d/%d tests passed (%.1f%%)\n", 
                passed_tests, total_tests, (float)passed_tests/total_tests*100);
}

void testPatternDetection() {
  Serial.println("\n=== Pattern Detection Test ===");
  
  // Simulate rhythmic wind pattern
  Serial.println("Simulating rhythmic wind pattern...");
  for (int i = 0; i < 20; i++) {
    float phase = i * 0.314; // 50ms * 20 = 1 second of data
    float wind_x = 3.0 * sin(phase * 2); // 2 Hz oscillation
    float wind_y = 2.5 * sin(phase * 2 + 0.5);
    float wind_z = 1.0 * sin(phase * 2 + 1.0);
    
    weatherFilter.addMotionSample(wind_x, wind_y, wind_z);
    delay(50);
  }
  
  MotionPattern detected = weatherFilter.classifyMotion();
  Serial.printf("Detected pattern: %s\n", 
                detected == PATTERN_WIND ? "WIND (correct)" : 
                detected == PATTERN_WILDLIFE ? "WILDLIFE (incorrect)" :
                detected == PATTERN_RAIN ? "RAIN (incorrect)" : "UNKNOWN");
  
  // Simulate random wildlife pattern
  Serial.println("Simulating random wildlife pattern...");
  weatherFilter.resetFilter(); // Clear previous data
  
  for (int i = 0; i < 10; i++) {
    float wildlife_x = random(-20, 20);
    float wildlife_y = random(-15, 15);
    float wildlife_z = random(-10, 25);
    
    weatherFilter.addMotionSample(wildlife_x, wildlife_y, wildlife_z);
    delay(100);
  }
  
  detected = weatherFilter.classifyMotion();
  Serial.printf("Detected pattern: %s\n", 
                detected == PATTERN_WILDLIFE ? "WILDLIFE (correct)" : 
                detected == PATTERN_WIND ? "WIND (incorrect)" :
                detected == PATTERN_RAIN ? "RAIN (incorrect)" : "UNKNOWN");
}

#endif // WEATHER_FILTERING_ENABLED

#if WILDLIFE_CAMERA_ENABLED

void testCameraMovement() {
  Serial.println("\n=== Camera Movement Test ===");
  
  Serial.println("Testing pan movement...");
  wildlifeCamera.panTo(45);
  delay(2000);
  wildlifeCamera.panTo(225);
  delay(2000);
  wildlifeCamera.panTo(135); // Return to center
  delay(1000);
  
  Serial.println("Testing tilt movement...");
  wildlifeCamera.tiltTo(45);
  delay(2000);
  wildlifeCamera.tiltTo(135);
  delay(2000);
  wildlifeCamera.tiltTo(90); // Return to level
  delay(1000);
  
  Serial.println("Testing coordinated movement...");
  wildlifeCamera.moveTo(90, 60);
  delay(2000);
  wildlifeCamera.moveTo(180, 120);
  delay(2000);
  wildlifeCamera.moveToHome();
  delay(1000);
  
  Serial.println("Camera movement test completed");
}

void testPowerManagement() {
  Serial.println("\n=== Power Management Test ===");
  
  PowerStatus status = wildlifeCamera.getPowerStatus();
  Serial.printf("Battery: %.2fV (%d%%)\n", status.battery_voltage, status.battery_percentage);
  Serial.printf("Solar: %.2fV %s\n", status.solar_voltage, status.is_charging ? "(CHARGING)" : "");
  Serial.printf("Status: %s\n", status.low_battery ? "LOW" : status.critical_battery ? "CRITICAL" : "GOOD");
  
  // Test power save mode simulation
  Serial.println("Simulating low battery condition...");
  // This would normally be triggered by actual low voltage
  // For testing, we can manually call the power optimization
  wildlifeCamera.optimizePowerConsumption();
  
  Serial.println("Power management test completed");
}

void testAreaSweep() {
  Serial.println("\n=== Area Sweep Test ===");
  
  Serial.println("Performing 90-degree sweep at level position...");
  wildlifeCamera.sweepArea(90, 180, 90, 15); // 90° to 180° in 15° steps
  
  Serial.println("Returning to home position...");
  wildlifeCamera.moveToHome();
  
  Serial.println("Area sweep test completed");
}

#endif // WILDLIFE_CAMERA_ENABLED

void runSystemTests() {
  Serial.println("\n╔══════════════════════════════════════════╗");
  Serial.println("║        ESP32-AI-CAM System Tests        ║");
  Serial.println("╚══════════════════════════════════════════╝");
  
#if WEATHER_FILTERING_ENABLED
  testWeatherFiltering();
  testPatternDetection();
#endif

#if WILDLIFE_CAMERA_ENABLED
  testCameraMovement();
  testPowerManagement();
  testAreaSweep();
#endif

  Serial.println("\n=== System Test Summary ===");
  Serial.printf("Weather Filtering: %s\n", WEATHER_FILTERING_ENABLED ? "ENABLED" : "DISABLED");
  Serial.printf("Wildlife Camera: %s\n", WILDLIFE_CAMERA_ENABLED ? "ENABLED" : "DISABLED");
  Serial.printf("Solar Power Monitoring: %s\n", SOLAR_POWER_MONITORING ? "ENABLED" : "DISABLED");
  Serial.printf("Pan/Tilt Servos: %s\n", PAN_TILT_SERVOS_ENABLED ? "ENABLED" : "DISABLED");
  Serial.printf("LoRa Communication: %s\n", LORA_COMMUNICATION_ENABLED ? "ENABLED" : "DISABLED");
  
  Serial.println("\nAll system tests completed!");
}

// Diagnostic functions
void printSystemDiagnostics() {
  Serial.println("\n=== System Diagnostics ===");
  
#if WEATHER_FILTERING_ENABLED
  weatherFilter.printDiagnostics();
#endif

#if WILDLIFE_CAMERA_ENABLED
  wildlifeCamera.printStatus();
#endif

  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Uptime: %lu seconds\n", millis() / 1000);
}

void printConfigurationSummary() {
  Serial.println("\n=== Configuration Summary ===");
  Serial.printf("Camera Home Position: Pan=%d°, Tilt=%d°\n", DEFAULT_PAN_HOME, DEFAULT_TILT_HOME);
  Serial.printf("Weather Filter Sensitivity: %.1f\n", WEATHER_FILTER_SENSITIVITY);
  Serial.printf("Wind Detection Threshold: %.1f\n", WIND_DETECTION_THRESHOLD);
  Serial.printf("Rain Detection Threshold: %.1f\n", RAIN_DETECTION_THRESHOLD);
  Serial.printf("Battery Capacity: %d mAh\n", BATTERY_CAPACITY_MAH);
  Serial.printf("Sweep Interval: %d hours\n", SWEEP_INTERVAL_HOURS);
  Serial.printf("Pictures per trigger: %d\n", number_of_pictures);
  Serial.printf("Time between pictures: %d ms\n", ms_between_pictures);
}