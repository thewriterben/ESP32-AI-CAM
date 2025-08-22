# Wildlife Trail Camera System Documentation

## Project Overview

This project transforms the original ESP32-AI-CAM into a comprehensive wildlife trail camera system using the LilyGO T-Camera Plus S3 platform. The system integrates solar power, automated pan/tilt tracking, PIR motion detection, LoRa mesh networking, and AI-powered wildlife identification for extended autonomous operation in field conditions.

## System Architecture

### Hardware Components

#### Core Processing Unit
- **LilyGO T-Camera Plus S3**: Main controller with integrated camera
- **ESP32-S3**: Dual-core processor with WiFi capability
- **OV5640 Camera**: 5MP camera with autofocus
- **PSRAM**: 8MB for image processing and AI inference

#### Power System
- **18650 Li-ion Battery**: 3.7V, 2000-3000mAh capacity
- **Solar Panel**: 6V, 2W monocrystalline panel
- **Charge Controller**: Integrated MPPT charging circuit
- **Power Management**: Automated power distribution and conservation

#### Motion Detection and Tracking
- **PIR Motion Sensor**: Passive infrared for wildlife detection
- **Pan/Tilt Servos**: 2x SG90 micro servos for 270°/180° coverage
- **Servo Control**: Precision positioning and tracking algorithms

#### Communication
- **LoRa SX1276**: Long-range communication module (915MHz)
- **WiFi**: Local connectivity for configuration and data upload
- **Mesh Networking**: Inter-camera communication and coordination

#### Environmental Monitoring
- **BME280**: Temperature, humidity, and pressure sensor
- **TSL2561**: Light level sensor for day/night detection
- **ADXL345**: Accelerometer for vibration detection

#### Storage and Interface
- **microSD Card**: Local image and data storage
- **Status LEDs**: System status indication
- **Configuration Interface**: Web-based setup and monitoring

### Software Architecture

#### Core Modules

1. **T-Camera Plus S3 Configuration** (`tcamera_plus_s3_config.h`)
   - Hardware pin definitions
   - System parameters and constants
   - GPIO assignments for all peripherals

2. **Servo Control Module** (`wildlife_servo_control.h/cpp`)
   - Pan/tilt servo management
   - Automated scanning patterns
   - Precision tracking algorithms
   - Power-saving servo control

3. **PIR Motion Detection** (`wildlife_pir_motion.h/cpp`)
   - Advanced motion filtering
   - Wildlife-specific pattern recognition
   - False positive reduction
   - Motion event analysis

4. **LoRa Communication** (`wildlife_lora_comm.h/cpp`)
   - Mesh network implementation
   - Message routing and forwarding
   - Wildlife detection alerts
   - Status reporting and coordination

5. **Power Management** (`wildlife_power_management.h/cpp`)
   - Solar charging optimization
   - Battery monitoring and protection
   - Adaptive power conservation
   - Deep sleep management

6. **AI Detection System** (`wildlife_ai_detection.h/cpp`)
   - Computer vision wildlife detection
   - Species classification
   - Behavioral analysis
   - Tracking integration

## Installation and Setup

### Hardware Assembly

#### 1. Main Unit Assembly
```
Components needed:
- LilyGO T-Camera Plus S3
- 18650 battery holder
- Solar panel with leads
- PIR motion sensor
- 2x SG90 servo motors
- LoRa SX1276 module
- Environmental sensors
- 3D printed enclosure parts
```

#### 2. Wiring Connections

**Power System:**
```
Solar Panel (+) → Charge Controller VIN
Solar Panel (-) → Ground
Battery (+) → Battery Input
Battery (-) → Ground
3.3V Rail → T-Camera Plus S3 VCC
5V Rail → Servo Power (via boost converter)
```

**GPIO Connections:**
```
GPIO 1  → PIR Motion Sensor OUT
GPIO 2  → Pan Servo Signal
GPIO 3  → Tilt Servo Signal
GPIO 14 → IR LED Array Control
GPIO 19 → Battery Voltage Monitor
GPIO 20 → LoRa CS
GPIO 21 → LoRa RST
GPIO 47 → LoRa DIO0
GPIO 48 → Light Sensor (LDR)
GPIO 38 → Status LED
GPIO 41 → I2C SDA (Sensors)
GPIO 42 → I2C SCL (Sensors)
```

**Camera Connections (T-Camera Plus S3):**
```
GPIO 15 → XCLK
GPIO 4  → SIOD (SDA)
GPIO 5  → SIOC (SCL)
GPIO 16 → Y9
GPIO 17 → Y8
GPIO 18 → Y7
GPIO 12 → Y6
GPIO 10 → Y5
GPIO 8  → Y4
GPIO 9  → Y3
GPIO 11 → Y2
GPIO 6  → VSYNC
GPIO 7  → HREF
GPIO 13 → PCLK
```

#### 3. Enclosure Assembly

1. **Print 3D enclosure parts** using specifications in `3D_ENCLOSURE_DESIGN.md`
2. **Install threaded inserts** in mounting holes
3. **Mount electronics** in designated compartments
4. **Install gaskets and seals** for weatherproofing
5. **Test mechanical operation** of pan/tilt mechanism
6. **Verify cable routing** and strain relief

### Software Installation

#### 1. Development Environment Setup

**Arduino IDE Configuration:**
```
1. Install Arduino IDE 1.8.19 or newer
2. Add ESP32 board support:
   - File → Preferences
   - Additional Board Manager URLs: 
     https://dl.espressif.com/dl/package_esp32_index.json
3. Install ESP32 board package version 2.0.5+
4. Select board: "ESP32S3 Dev Module"
```

**Required Libraries:**
```
Core Libraries:
- ESP32Servo by Kevin Harrington v0.12.1+
- LoRa by Sandeep Mistry v0.8.0+

Sensor Libraries:
- SparkFun BME280 by SparkFun v2.0.9+
- SparkFun TSL2561 by SparkFun v1.1.0+
- SparkFun ADXL345 by SparkFun v1.0.0+

Cloud Integration (Optional):
- AzureIoTHub by Microsoft v1.6.0+
- ArduinoJson by Benoit Blanchon v6.19.4+
```

#### 2. Configuration

**Update settings.h:**
```cpp
// WiFi Configuration
#define IOT_CONFIG_WIFI_SSID "YourNetworkName"
#define IOT_CONFIG_WIFI_PASSWORD "YourPassword"

// Camera Settings
#define ms_between_pictures 5000
#define number_of_pictures 3

// LoRa Configuration
#define CAMERA_ID 1  // Unique ID for this camera

// Power Management
#define BATTERY_CAPACITY_MAH 2500
#define SOLAR_PANEL_WATTS 2.0
```

**Customize AI Detection:**
```cpp
// In wildlife_ai_detection.cpp
config.detectionThreshold = 70;     // Minimum confidence for detection
config.speciesThreshold = 60;       // Minimum confidence for species ID
config.enableTracking = true;       // Enable object tracking
config.enableBehaviorAnalysis = true; // Analyze behavior patterns
```

#### 3. Compilation and Upload

```
1. Select correct board: "ESP32S3 Dev Module"
2. Set partition scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"
3. Set PSRAM: "OPI PSRAM"
4. Set upload speed: 921600
5. Connect T-Camera Plus S3 via USB
6. Hold BOOT button while clicking upload
7. Release BOOT button when upload starts
```

## Field Deployment

### Site Selection

**Optimal Placement:**
```
- Unobstructed solar panel exposure (6+ hours daily sun)
- Clear camera view of wildlife corridors
- Stable mounting surface (tree, post, etc.)
- LoRa communication range to other units
- Protection from direct weather exposure
```

**Mounting Considerations:**
```
- Height: 2-3 meters for optimal coverage
- Angle: Slight downward tilt for ground-level wildlife
- Security: Anti-theft mounting hardware
- Accessibility: Maintenance access without tools
```

### Initial Configuration

#### 1. Power System Verification
```
1. Install fully charged 18650 battery
2. Verify solar panel connection and voltage
3. Check battery charging status LED
4. Monitor power consumption in serial output
```

#### 2. Motion Detection Calibration
```
1. Allow PIR sensor 30 seconds to stabilize
2. Test motion detection range and sensitivity
3. Adjust PIR settings for local conditions
4. Verify false positive filtering
```

#### 3. Servo Calibration
```
1. Test pan/tilt range of motion
2. Verify home position accuracy
3. Check tracking smoothness and speed
4. Calibrate position feedback
```

#### 4. LoRa Network Setup
```
1. Set unique camera ID (1-255)
2. Configure frequency for your region
3. Test communication with other units
4. Verify mesh network formation
```

### Operation Modes

#### Normal Operation
- **Motion-Triggered Recording**: Activated by PIR detection
- **Scheduled Scanning**: Periodic area surveillance
- **Real-Time Monitoring**: Continuous operation when powered
- **Network Coordination**: Multi-camera synchronization

#### Power Conservation Mode
- **Reduced CPU Frequency**: Lower processing speed
- **Servo Power Management**: Disable when not in use
- **LoRa Duty Cycling**: Reduced transmission frequency
- **Adaptive Sensing**: Adjust based on activity levels

#### Emergency Mode
- **Critical Battery Protection**: Preserve minimum power
- **Essential Functions Only**: Motion detection and communication
- **Extended Deep Sleep**: Wake only for critical events
- **Low Battery Alerts**: Notify maintenance requirements

## System Monitoring and Maintenance

### Real-Time Monitoring

**Serial Output Monitoring:**
```
=== Wildlife Camera System Status ===
Power State: 0 (Normal)
Battery: 3.85V (78%)
Solar: 6.2V (1.8W)
Charging: Yes
Runtime: 1440 minutes
Motion Events: 12
Wildlife Detections: 8
LoRa Messages: 156
Network Cameras: 3
```

**LoRa Network Status:**
```
Camera 1: Battery=3.8V, Temp=22°C, RSSI=-67dBm
Camera 2: Battery=4.0V, Temp=18°C, RSSI=-73dBm
Camera 3: Battery=3.6V, Temp=25°C, RSSI=-81dBm
```

### Maintenance Schedule

#### Daily (Automated)
- Battery voltage monitoring
- Solar charging verification
- Motion detection statistics
- Network connectivity check

#### Weekly (Remote)
- Wildlife detection review
- Power consumption analysis
- Image quality assessment
- Network performance evaluation

#### Monthly (Field Visit)
- Solar panel cleaning
- Physical inspection of seals
- Battery condition check
- Servo operation verification

#### Seasonal (Major Service)
- Complete system inspection
- Software updates
- Hardware component replacement
- Calibration verification

### Troubleshooting

#### Common Issues

**Power Problems:**
```
Issue: Battery drains quickly
Solutions:
- Check solar panel positioning and cleanliness
- Verify power consumption in serial output
- Enable aggressive power conservation mode
- Check for damaged cables or connections
```

**Motion Detection Issues:**
```
Issue: Too many false positives
Solutions:
- Adjust PIR sensitivity settings
- Enable wind filtering
- Check for vegetation movement in view
- Verify mounting stability
```

**Communication Problems:**
```
Issue: LoRa messages not received
Solutions:
- Check antenna connection and positioning
- Verify frequency configuration
- Test with shorter range
- Check for interference sources
```

**Camera Problems:**
```
Issue: Poor image quality
Solutions:
- Clean camera lens
- Check for condensation in enclosure
- Verify adequate lighting
- Adjust camera settings in code
```

## Performance Optimization

### Battery Life Optimization

**Expected Performance:**
- **Normal Operation**: 2-4 weeks without solar
- **With Solar**: Indefinite operation with 4+ hours daily sun
- **Conservation Mode**: 6-8 weeks without solar
- **Emergency Mode**: 10-12 weeks minimal operation

**Optimization Strategies:**
```cpp
// Aggressive power conservation
powerManager.setConservationMode(true);
pirMotion.setMotionSensitivity(60);  // Reduce sensitivity
aiDetection.setDetectionThreshold(80);  // Higher threshold

// Optimize LoRa duty cycle
loraComm.setTransmissionPower(14);  // Reduce from 17dBm
// Increase heartbeat interval to 10 minutes
```

### Detection Accuracy Tuning

**AI Model Optimization:**
```cpp
// Adjust confidence thresholds based on field testing
aiDetection.setDetectionThreshold(75);    // Higher for fewer false positives
aiDetection.setSpeciesThreshold(65);      // Balanced species identification

// Enable behavioral analysis for better classification
config.enableBehaviorAnalysis = true;
config.enableTracking = true;
```

**Motion Detection Tuning:**
```cpp
// Reduce wind-induced false positives
pirMotion.enableWindFiltering(true);
pirMotion.setMotionSensitivity(70);      // Adjust based on local conditions
pirMotion.setConfidenceThreshold(75);    // Require higher confidence
```

### Network Performance

**LoRa Range Optimization:**
```cpp
// Maximize range with higher power and lower data rate
LoRa.setTxPower(20);                    // Maximum power
LoRa.setSpreadingFactor(12);            // Maximum range
LoRa.setSignalBandwidth(125E3);         // Standard bandwidth
LoRa.setCodingRate4(8);                 // Maximum error correction
```

**Mesh Network Efficiency:**
```cpp
// Optimize message routing
loraComm.setHeartbeatInterval(300000);  // 5 minutes
loraComm.enableMessageCompression(true);
loraComm.setMaxRetries(2);              // Reduce retries to save power
```

## Integration with Cloud Services

### Azure IoT Hub Integration

The system maintains compatibility with Azure IoT Hub for cloud-based monitoring and analysis:

```cpp
// Original Azure functionality preserved
// Wildlife detection data uploaded to blob storage
// Environmental telemetry sent via IoT Hub
// Remote configuration and control capabilities
```

### Data Export and Analysis

**Wildlife Detection Data:**
```json
{
  "timestamp": "2023-10-15T14:30:00Z",
  "camera_id": 1,
  "species": "MAMMAL_MEDIUM",
  "confidence": 85,
  "location": {"x": 0.4, "y": 0.6, "width": 0.3, "height": 0.2},
  "behavior": {"moving": true, "feeding": false, "alert": true},
  "environment": {"temperature": 22, "humidity": 65, "light": 75},
  "battery_voltage": 3.85,
  "image_url": "https://storage.blob.core.windows.net/images/..."
}
```

**Network Statistics:**
```json
{
  "network_id": "wildlife_mesh_001",
  "active_cameras": 3,
  "total_detections": 156,
  "wildlife_confirmed": 98,
  "false_positives": 12,
  "average_rssi": -72,
  "network_health": 95
}
```

This comprehensive wildlife trail camera system provides autonomous, long-term wildlife monitoring with advanced AI detection, solar power operation, and mesh networking capabilities for scientific research and conservation applications.