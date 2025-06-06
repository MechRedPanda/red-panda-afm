# DIY AFM Building Instructions

## Overview
This guide will walk you through building your own low-cost Atomic Force Microscope (AFM) using 3D printed components and readily available electronic parts.

## Required Materials

### 3D Printed Components
- Main frame structure
- Sample stage
- Cantilever mount
- Optical head assembly housing
- Z-axis adjustment mechanism

### Electronic Components
- **OPU (Optical Pick-Up Unit)**: DVD/Blu-ray laser assembly
- **AFM Probe/Cantilever**: Silicon nitride or silicon cantilever
- **Photodiode**: For detecting cantilever deflection
- **Piezoelectric actuators**: For fine positioning (X, Y, Z axes)
- **Arduino/microcontroller**: For control and data acquisition
- **Operational amplifiers**: For signal conditioning
- **Resistors and capacitors**: Various values for circuit construction

### Tools Required
- Soldering iron and solder
- Multimeter
- Small screwdrivers (Phillips and flathead)
- Wire strippers
- Hot glue gun (optional)
- Computer with USB port

## Assembly Steps

### Step 1: 3D Print Components
1. Download and print all STL files using PLA or PETG filament
2. Use 0.2mm layer height for best results
3. Support material may be required for some components
4. Post-process printed parts by removing supports and light sanding if needed

### Step 2: Prepare Electronics
1. **OPU Preparation**:
   - Carefully disassemble DVD/Blu-ray drive to extract OPU
   - Identify laser diode pins and photodiode connections
   - Test laser functionality with appropriate current limiting

2. **Control Circuit Assembly**:
   - Solder operational amplifier circuit for photodiode signal conditioning
   - Connect piezoelectric actuators to driver circuits
   - Wire Arduino/microcontroller to control interfaces

### Step 3: Mechanical Assembly
1. **Frame Assembly**:
   - Assemble main frame components
   - Install sample stage with smooth movement
   - Mount Z-axis adjustment mechanism

2. **Optical System Setup**:
   - Install OPU in optical head housing
   - Align laser beam path
   - Position photodiode for optimal signal detection

3. **Cantilever Mount**:
   - Install AFM probe/cantilever in mount
   - Ensure proper alignment with laser beam
   - Test mechanical stability

### Step 4: Electronics Integration
1. Connect all electronic components according to circuit diagram
2. Install control software on computer
3. Test all actuators and sensors
4. Calibrate photodiode response

### Step 5: Initial Testing
1. **System Check**:
   - Verify all connections
   - Test laser safety (never look directly into beam)
   - Check piezo actuator response

2. **Basic Functionality Test**:
   - Load test sample (e.g., calibration grating)
   - Attempt basic scanning operation
   - Verify data acquisition

## Safety Considerations

⚠️ **IMPORTANT SAFETY NOTES**:
- **Laser Safety**: Never look directly into the laser beam
- **Electrical Safety**: Ensure proper grounding and current limiting
- **Mechanical Safety**: Be careful with sharp AFM probes
- **Work in well-lit area** with proper ventilation when soldering

## Calibration

### Initial Calibration Steps
1. **Optical Alignment**:
   - Align laser spot on cantilever
   - Center photodiode signal
   - Optimize signal-to-noise ratio

2. **Mechanical Calibration**:
   - Calibrate piezo actuator sensitivity
   - Set appropriate feedback parameters
   - Test approach/retract functionality

## Troubleshooting

### Common Issues
- **No laser signal**: Check laser diode connections and current limiting
- **Poor image quality**: Verify cantilever alignment and cleanliness
- **Instability**: Check for vibrations and improve isolation
- **Electrical noise**: Improve shielding and grounding

### Performance Optimization
- Use vibration isolation table if available
- Shield electronics from electromagnetic interference
- Keep cables short and well-routed
- Regular maintenance of moving parts

## Software Setup
1. Install required drivers for microcontroller
2. Download AFM control software
3. Configure communication parameters
4. Test data acquisition and display

## Expected Performance
- **Lateral resolution**: ~100 nm (depending on probe quality)
- **Vertical resolution**: ~1 nm
- **Scan range**: Limited by piezo actuator range
- **Scan speed**: Variable, typically 0.1-10 Hz

## Next Steps
- Start with simple test samples (calibration gratings)
- Gradually move to more complex samples
- Consider upgrading components for better performance
- Join AFM community forums for tips and troubleshooting

## Support
For additional help and community support, refer to:
- Project repository issues section
- AFM hobbyist forums
- Local maker spaces with electronics expertise

---
**Note**: This is a complex project requiring patience and attention to detail. Take your time with each step and don't hesitate to ask for help when needed. 