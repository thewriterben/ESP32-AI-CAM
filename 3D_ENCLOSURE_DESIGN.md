# Wildlife Trail Camera 3D Enclosure Design Specifications

## Overview
This document specifies the design requirements for 3D printable enclosures for the wildlife trail camera system using LilyGO T-Camera Plus S3 units integrated with solar panels and weatherproofing.

## Design Requirements

### General Specifications
- **IP Rating**: IP67 (dust-tight and waterproof to 1m depth for 30 minutes)
- **Operating Temperature**: -20°C to +60°C
- **Material**: UV-resistant PETG or ABS plastic
- **Print Settings**: 0.2mm layer height, 20% infill minimum
- **Assembly**: Tool-free wherever possible, captured hardware

### Main Enclosure Dimensions
```
External Dimensions: 200mm (L) x 150mm (W) x 100mm (H)
Wall Thickness: 3mm minimum
Corner Radius: 5mm for stress distribution
```

### Component Layout

#### Main Housing (housing_main.stl)
```
Internal Compartments:
- T-Camera Plus S3 compartment: 85mm x 65mm x 20mm
- Battery compartment: 70mm x 20mm x 70mm (18650 battery)
- Power management compartment: 60mm x 40mm x 30mm
- LoRa module space: 40mm x 30mm x 15mm
- Servo mounting wells: 2x 25mm x 25mm x 30mm
```

#### Camera Gimbal Housing (gimbal_housing.stl)
```
Specifications:
- Pan range: 270° continuous rotation
- Tilt range: 180° (±90° from horizontal)
- Servo mounting: Standard SG90 servo mounts
- Bearing races: 8mm ball bearing compatible
- Camera window: 30mm diameter clear polycarbonate
- Anti-fog venting: Micro breathable membrane
```

#### Solar Panel Mount (solar_mount.stl)
```
Specifications:
- Panel size: 110mm x 110mm (standard 2W panel)
- Tilt adjustment: 0° to 90° in 15° increments
- Locking mechanism: Thumb screw retention
- Cable entry: IP67 rated gland seal
- Mounting pattern: Compatible with standard solar panels
```

#### Weatherproof Base (base_plate.stl)
```
Specifications:
- Mounting holes: 4x M8 threaded inserts
- Drainage channels: 5mm wide, 2mm deep
- Cable entry ports: 4x 12mm diameter with seal glands
- Ventilation valve: One-way moisture relief
- Anti-theft mounting: Security screw accessible
```

## Detailed Component Designs

### 1. Main Housing Assembly

#### Part: housing_main.stl
```yaml
Features:
  - Split-shell design (top/bottom halves)
  - O-ring groove: 2mm wide x 1.5mm deep
  - Captive thumb screws: 4x M6 x 20mm
  - Internal ribs for structural strength
  - Component mounting posts with threaded inserts
  
Printing Notes:
  - Print orientation: Bottom shell face-down
  - Support material: Minimal, for overhangs >45°
  - Post-processing: Drill out seal grooves to exact dimensions
```

#### Part: housing_lid.stl
```yaml
Features:
  - Clear window for status LED visibility
  - Antenna pass-through with seal
  - Heat sink mounting for power electronics
  - Internal cable management clips
  
Special Requirements:
  - Must mate perfectly with housing_main
  - O-ring compression: 15-20%
```

### 2. Camera Gimbal System

#### Part: gimbal_housing.stl
```yaml
Features:
  - Two-axis gimbal design
  - Integrated servo mounting brackets
  - Ball bearing races (8mm x 22mm x 7mm)
  - Camera mounting platform
  - Cable management through center axis
  
Tolerances:
  - Bearing fits: H7/h6 (slip fit)
  - Servo mounting: ±0.1mm
  - Rotation clearance: 0.5mm minimum
```

#### Part: camera_platform.stl
```yaml
Features:
  - T-Camera Plus S3 mounting holes
  - Vibration dampening ribs
  - Lens protection hood
  - Cable strain relief
  
Camera Window Specifications:
  - Material: 3mm polycarbonate disc
  - Diameter: 30mm
  - UV protection coating
  - Anti-reflective treatment
```

### 3. Solar Panel Integration

#### Part: solar_mount.stl
```yaml
Features:
  - Adjustable tilt mechanism
  - Standard solar panel mounting holes
  - Weather seal channel
  - Cable management
  
Adjustment Mechanism:
  - Detent positions every 15°
  - Spring-loaded positioning pin
  - Thumb screw lock
  - Maximum load: 5N wind force
```

#### Part: solar_bracket.stl
```yaml
Features:
  - Connects solar mount to main housing
  - Allows 360° rotation for optimal positioning
  - Locking collar with friction fitting
  - Integrated cable routing
```

### 4. Mounting and Security

#### Part: tree_mount.stl
```yaml
Features:
  - Universal tree mounting clamp
  - Adjustable diameter: 50mm to 200mm
  - Stainless steel strap compatibility
  - Security screw access (requires special tool)
  - Level adjustment: ±15° in all directions
```

#### Part: post_mount.stl
```yaml
Features:
  - Standard fence post mounting
  - Compatible with 75mm and 100mm posts
  - U-bolt mounting system
  - Anti-rotation tabs
  - Theft-deterrent design
```

## Weatherproofing Details

### Seal Systems
```yaml
Primary Seal:
  - O-ring: Nitrile rubber, 2mm cross-section
  - Groove: AS568 standard
  - Compression: 15-20%
  
Secondary Seals:
  - Cable glands: IP67 rated, M12 thread
  - Vent membrane: Breathable, waterproof
  - Lens gasket: Silicone, custom profile
```

### Drainage and Ventilation
```yaml
Condensation Management:
  - Internal moisture absorber compartment
  - Breathable membrane for pressure equalization
  - Drainage channels with weep holes
  - Desiccant pouch mounting points
```

## Materials and Hardware

### 3D Printing Materials
```yaml
Primary Choice: PETG
  - UV resistance: Excellent
  - Temperature range: -20°C to +60°C
  - Chemical resistance: Good
  - Printing difficulty: Medium
  
Alternative: ABS
  - UV resistance: Good (with additives)
  - Temperature range: -40°C to +80°C
  - Strength: Higher than PETG
  - Printing difficulty: Higher (requires heated chamber)
```

### Hardware Bill of Materials
```yaml
Fasteners:
  - M6 x 20mm thumb screws: 4 pieces
  - M4 x 12mm cap screws: 8 pieces
  - M3 x 8mm cap screws: 12 pieces
  - M8 threaded inserts: 4 pieces
  - M4 threaded inserts: 8 pieces
  
Seals and Gaskets:
  - O-ring 2mm x 100mm: 1 piece
  - Cable glands M12: 4 pieces
  - Silicone gasket sheet: 200mm x 200mm
  
Bearings and Hardware:
  - Ball bearings 8x22x7mm: 2 pieces
  - Torsion springs: 2 pieces
  - Stainless steel pins: 4 pieces
```

## Assembly Instructions

### Pre-Assembly Preparation
1. **Post-Processing**
   - Remove all support material
   - Sand critical mating surfaces
   - Drill and tap threaded insert holes
   - Test fit all components

2. **Hardware Installation**
   - Install threaded inserts using heat-set tool
   - Apply thread locker to permanent fasteners
   - Install bearings with light press fit

### Main Assembly Sequence
1. **Electronics Installation**
   - Mount T-Camera Plus S3 to platform
   - Install power management module
   - Route cables through strain reliefs
   - Install LoRa antenna with seal

2. **Gimbal Assembly**
   - Install servos in mounting brackets
   - Mount camera platform to tilt mechanism
   - Install pan mechanism with bearings
   - Test full range of motion

3. **Enclosure Assembly**
   - Install O-ring in groove
   - Place electronics in bottom shell
   - Route cables through glands
   - Install top shell with proper torque

4. **Solar Panel Installation**
   - Mount solar panel to bracket
   - Install adjustment mechanism
   - Connect power cables
   - Test positioning range

## Quality Control and Testing

### Fit and Function Tests
```yaml
Mechanical Tests:
  - Servo range verification
  - Seal compression check
  - Hardware torque verification
  - Thermal expansion test
  
Environmental Tests:
  - Water immersion test (IP67)
  - Temperature cycling (-20°C to +60°C)
  - UV exposure test (500 hours)
  - Vibration resistance test
```

### Field Testing Protocol
```yaml
Installation Test:
  - Mounting system stability
  - Solar panel positioning
  - Weatherproofing verification
  - System functionality check
  
Long-term Testing:
  - 30-day outdoor exposure
  - Battery performance monitoring
  - Image quality assessment
  - Maintenance requirement evaluation
```

## Maintenance and Serviceability

### Regular Maintenance (Monthly)
- Visual inspection of seals
- Solar panel cleaning
- Battery voltage check
- Servo operation verification

### Annual Maintenance
- O-ring replacement
- Desiccant replacement
- Bearing lubrication
- Hardware torque verification

### Service Access
- Tool-free battery compartment access
- Quick-disconnect camera module
- Modular component replacement
- Diagnostic port accessibility

## Manufacturing Notes

### Print Settings Recommendations
```yaml
Layer Height: 0.2mm (0.15mm for critical surfaces)
Infill: 20% minimum (30% for structural components)
Wall Thickness: 3 perimeters minimum
Support: Enable for overhangs >45°
Print Speed: 60mm/s maximum for quality
```

### Quality Control Checkpoints
1. Dimensional verification with calipers
2. Surface finish inspection
3. Threaded insert installation verification
4. Assembly fit test before finishing

This comprehensive design specification ensures the 3D printed enclosure will provide reliable protection for the wildlife trail camera system while maintaining functionality and ease of maintenance.