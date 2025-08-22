/*
  ESP32-AI-CAM Wildlife Camera System - Test Functions
  
  Header file for test and validation functions
*/

#ifndef SYSTEM_TESTS_H
#define SYSTEM_TESTS_H

#include <Arduino.h>

// Test function declarations
void runSystemTests();
void printSystemDiagnostics();
void printConfigurationSummary();

#if WEATHER_FILTERING_ENABLED
void testWeatherFiltering();
void testPatternDetection();
#endif

#if WILDLIFE_CAMERA_ENABLED
void testCameraMovement();
void testPowerManagement();
void testAreaSweep();
#endif

#endif // SYSTEM_TESTS_H