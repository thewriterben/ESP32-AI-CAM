# Wildlife Trail Camera System - Weather Motion Filtering

This repository now includes comprehensive weather motion filtering capabilities to eliminate false triggers from environmental conditions.

## New Features Added

### 1. Weather Motion Filtering (`weather_filter.h/cpp`)

**Comprehensive filtering system that eliminates false triggers from:**
- **Wind-Related Motion**: Rhythmic swaying branches, leaf movement, debris
- **Water-Related Motion**: Rain droplets, water ripples, dripping patterns  
- **Weather Events**: Snow, fog, lightning, storm conditions
- **Environmental Changes**: Shadow movement, temperature fluctuations

**Key Features:**
- Multi-sensor fusion using BME280 (temp/humidity/pressure), TSL2561 (light), and ADXL345 (motion)
- Temporal pattern analysis to distinguish wildlife vs environmental motion
- Adaptive sensitivity adjustment based on weather conditions
- Real-time environmental condition monitoring
- Machine learning-style pattern classification

### 2. Wildlife Camera Extensions (`wildlife_camera.h/cpp`)

**Advanced camera control system:**
- **Pan/Tilt Control**: 270° horizontal, 180° vertical coverage with smooth servo movement
- **Solar Power Management**: Continuous monitoring with adaptive power saving
- **IR Illumination**: Automatic day/night switching with brightness control
- **LoRa Communication**: Inter-camera coordination and remote control framework
- **Power Optimization**: Multiple power saving modes for extended operation

### 3. Enhanced Motion Detection

**The ADXL345 interrupt handler now includes:**
- Real-time weather filtering before triggering camera
- Environmental context awareness
- Motion pattern classification
- Detailed logging of filter decisions

## Technical Implementation

### Weather Filtering Algorithm

1. **Motion Sample Collection**: Continuously samples accelerometer data into rolling buffer
2. **Environmental Monitoring**: Tracks temperature, humidity, pressure, and light levels
3. **Pattern Analysis**: 
   - Calculates motion correlation to detect rhythmic patterns (wind)
   - Analyzes frequency content to identify water droplets vs wildlife
   - Detects vegetation sway through horizontal motion dominance
4. **Environmental Context**: Considers current weather conditions in classification
5. **Adaptive Thresholds**: Automatically adjusts sensitivity based on conditions

### Multi-Sensor Validation

The system uses multiple sensors to validate motion triggers:
- **Accelerometer**: Primary motion detection
- **Barometric Pressure**: Weather change detection  
- **Humidity**: Rain/fog detection
- **Light Level**: Storm/night condition detection
- **Temperature**: Rapid weather change detection

### Power Management

- **Solar Monitoring**: Continuous voltage monitoring with charging detection
- **Battery Management**: Multi-level power saving (low battery → critical battery → deep sleep)
- **Servo Control**: Automatic detachment during power save mode
- **IR Optimization**: Automatic brightness adjustment based on battery level

## Usage

The system automatically initializes during ESP32 startup and begins filtering motion events. All motion triggers now pass through the weather filter before activating the camera.

**Key Functions:**
- `isMotionFiltered(x, y, z)` - Main filtering function
- `weatherFilter.printDiagnostics()` - Detailed filter status
- `wildlifeCamera.performScheduledTasks()` - Power management and maintenance
- `wildlifeCamera.performPeriodicSweep()` - Automated area scanning

## Configuration

Key parameters can be adjusted in the header files:
- `MOTION_SAMPLE_WINDOW` - Number of motion samples to analyze (default: 50)
- `WIND_PATTERN_THRESHOLD` - Correlation threshold for wind detection (default: 0.7)
- `RAIN_LIGHT_THRESHOLD` - Light level change indicating rain (default: 0.3)
- `BATTERY_LOW_VOLTAGE` - Low battery threshold (default: 3.4V)

## Hardware Requirements

- LilyGO T-Camera Plus S3 or ESP32-CAM
- BME280 environmental sensor
- TSL2561 light sensor  
- ADXL345 accelerometer
- Servo motors for pan/tilt (optional)
- Solar panel and battery management (optional)
- LoRa module for communication (optional)

## Example Output

```
Weather Filter: CLEAR
*** ACTIVITY *** (VALID WILDLIFE MOTION)
Wildlife pattern detected

Weather Filter: RAIN  
*** ACTIVITY *** (FILTERED - Weather/Environmental Motion)
Rain pattern detected - motion filtered
```

This implementation ensures that the trail camera only triggers on genuine wildlife movement, dramatically reducing false positives from environmental conditions.