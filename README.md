# FTC-2025 Robot Code

Repository for the 2025 FTC season swerve robot with turret, shooter, and intake system.

## Project Overview

This repository contains a complete FTC TeleOp program with modular subsystem architecture for a competition robot featuring:

- **Swerve Drivetrain** with field-centric control and absolute encoders
- **Turret & Shooter System** with auto-aim using Limelight/AprilTags
- **Intake & Sorting System** with color sensor detection

## Quick Start

See [TeamCode/README.md](TeamCode/README.md) for detailed documentation, hardware configuration, and calibration instructions.

## Project Structure

```
TeamCode/
├── src/main/java/org/firstinspires/ftc/teamcode/
│   ├── RobotHardware.java           # Hardware initialization
│   ├── subsystems/                  # Modular subsystems
│   │   ├── SwerveModule.java        # Individual swerve module
│   │   ├── SwerveDrive.java         # Swerve drive controller
│   │   ├── Turret.java              # Turret and shooter
│   │   └── Sorter.java              # Intake and sorting
│   └── opmodes/
│       └── TeleOpMain.java          # Main TeleOp OpMode
└── README.md                         # Detailed documentation
```

## Features

### Swerve Drive System
- Field-centric control for intuitive driving
- Rev Through Bore Encoders (Analog/Absolute) for precise module positioning
- PID-controlled steering for each module
- Optimized module states to minimize rotation

### Turret System
- Auto-aim using Limelight (AprilTag detection)
- Odometry fallback when target is lost
- Distance-based hood angle adjustment
- PID-controlled turret positioning

### Sorting System
- Real-time color detection with multiple sensors
- Automatic ball sorting with "lopata" (shovel) servos
- Alliance-based target color configuration

## Getting Started

1. Review [TeamCode/README.md](TeamCode/README.md) for complete documentation
2. Configure hardware devices in Robot Controller app
3. Calibrate encoder offsets and PID values
4. Test each subsystem individually before full integration
5. Deploy and run "TeleOpMain" from the Driver Station

## Code Statistics

- **Total Lines:** ~1300 lines of Java code
- **Classes:** 6 main classes
- **Subsystems:** 3 modular subsystems
- **Hardware Devices:** 20+ motors, servos, and sensors

## Requirements

- FTC SDK 9.0+ (or compatible version)
- Android Studio for development and deployment
- Rev Control Hub and Expansion Hub
- Robot hardware as specified in TeamCode/README.md

## License

This code is provided for FTC team use. Please follow FTC rules and regulations.