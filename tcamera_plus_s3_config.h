/*
  T-Camera Plus S3 Hardware Configuration
  
  Wildlife Trail Camera System Integration
  For LilyGO T-Camera Plus S3 units with solar power and wildlife detection
  
  Hardware Pin Assignments for Wildlife Trail Camera System
*/

#ifndef TCAMERA_PLUS_S3_CONFIG_H
#define TCAMERA_PLUS_S3_CONFIG_H

// T-Camera Plus S3 Camera Pin Definitions
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM     4
#define SIOC_GPIO_NUM     5

#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM       8
#define Y3_GPIO_NUM       9
#define Y2_GPIO_NUM       11
#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     7
#define PCLK_GPIO_NUM     13

// Wildlife Trail Camera System GPIO Assignments
#define PIR_MOTION_PIN          1    // PIR Motion Sensor
#define PAN_SERVO_PIN           2    // Pan Servo PWM
#define TILT_SERVO_PIN          3    // Tilt Servo PWM  
#define IR_LED_CONTROL_PIN      14   // IR LED Array Control (moved from GPIO 4)
#define BATTERY_MONITOR_PIN     19   // Battery Voltage Monitor (moved from GPIO 5)
#define LORA_CS_PIN             20   // LoRa CS
#define LORA_RST_PIN            21   // LoRa RST (moved I2C to different pins)
#define LORA_DIO0_PIN           47   // LoRa DIO0 (moved from GPIO 8)
#define LIGHT_SENSOR_PIN        48   // Light Sensor (LDR) (moved from GPIO 9)
#define STATUS_LED_PIN          38   // Status LED (moved from GPIO 10)

// I2C Pins for Power Management & Environmental Sensors
#define I2C_SDA_PIN             41   // I2C SDA (moved from GPIO 21)
#define I2C_SCL_PIN             42   // I2C SCL (moved from GPIO 22)

// SD Card SPI Pins (T-Camera Plus S3)
#define SD_CS_PIN               39
#define SD_MOSI_PIN             40
#define SD_MISO_PIN             43
#define SD_SCK_PIN              44

// Power Management
#define SOLAR_PANEL_ENABLE_PIN  45   // Solar panel charging enable
#define POWER_SAVE_MODE_PIN     46   // Power save mode control

// Servo Control Parameters
#define SERVO_MIN_PULSE         500   // Minimum pulse width in microseconds
#define SERVO_MAX_PULSE         2500  // Maximum pulse width in microseconds
#define SERVO_FREQUENCY         50    // Servo frequency in Hz
#define PAN_RANGE_DEGREES       270   // Pan servo range
#define TILT_RANGE_DEGREES      180   // Tilt servo range

// PIR Motion Detection
#define PIR_TRIGGER_DELAY       2000  // PIR trigger delay in milliseconds
#define PIR_DETECTION_RANGE     10    // Detection range in meters

// Power Management Settings
#define BATTERY_MIN_VOLTAGE     3.2   // Minimum battery voltage before deep sleep
#define BATTERY_MAX_VOLTAGE     4.2   // Maximum battery voltage (fully charged)
#define SOLAR_CHARGING_VOLTAGE  5.0   // Solar panel charging voltage threshold

// Wildlife Detection Parameters
#define MOTION_THRESHOLD        50    // Motion detection sensitivity
#define RECORDING_DURATION      30    // Recording duration in seconds
#define MAX_DETECTION_DISTANCE  15    // Maximum detection distance in meters

// LoRa Communication Settings
#define LORA_FREQUENCY          915E6 // LoRa frequency (915 MHz for North America)
#define LORA_BANDWIDTH          125E3 // LoRa bandwidth
#define LORA_SPREADING_FACTOR   7     // LoRa spreading factor
#define LORA_CODING_RATE        5     // LoRa coding rate
#define LORA_TX_POWER           17    // LoRa transmission power

// Camera Settings for Wildlife
#define CAMERA_RESOLUTION       FRAMESIZE_QSXGA  // High resolution for wildlife details
#define CAMERA_JPEG_QUALITY     8                // Balance between quality and size
#define NIGHT_VISION_THRESHOLD  50               // Light level threshold for night mode

#endif // TCAMERA_PLUS_S3_CONFIG_H