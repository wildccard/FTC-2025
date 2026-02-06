# FTC 2025 TeleOp Program Documentation

This repository contains a complete FTC TeleOp program with modular subsystem architecture for a swerve drive robot with turret, shooter, and sorting capabilities.

## Project Structure

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── RobotHardware.java          # Hardware initialization class
├── opmodes/
│   └── TeleOpMain.java         # Main TeleOp OpMode
└── subsystems/
    ├── SwerveDrive.java        # Swerve drivetrain subsystem
    ├── Turret.java             # Turret and shooter subsystem
    └── Sorter.java             # Intake and sorting subsystem
```

## Hardware Configuration

### Control Hub
- **Swerve Drivetrain:**
  - 4x Drive Motors: `driveFL`, `driveFR`, `driveBL`, `driveBR`
  - 4x Steering Servos (CR): `steerFL`, `steerFR`, `steerBL`, `steerBR`
  - 4x Steering Encoders (Analog): `encoderFL`, `encoderFR`, `encoderBL`, `encoderBR`
  - IMU: `imu`

### Expansion Hub
- **Turret & Shooter:**
  - 2x Shooter Motors: `shooter1`, `shooter2`
  - 2x Turret Servos: `turret1`, `turret2`
  - 1x Hood Servo: `hood`
  - 1x Turret Encoder (Analog): `turretEncoder`

- **Intake & Sorting:**
  - 2x Intake Motors: `intake1`, `intake2`
  - 3x Color Sensors: `colorSensor1`, `colorSensor2`, `colorSensor3`
  - 3x Lopata (Shovel) Servos: `lopata1`, `lopata2`, `lopata3`

## Subsystems

### SwerveDrive
Field-centric swerve drivetrain control with PID steering control for each module.

**Features:**
- Field-centric driving using IMU
- Individual module control with absolute encoders
- Optimized steering (shortest path to target angle)
- Configurable wheel base dimensions

**TODO:**
- Tune PID values (`STEERING_KP`, `STEERING_KI`, `STEERING_KD`)
- Calibrate encoder offsets for each module
- Measure and update wheel base dimensions

### Turret
Auto-aiming turret system with shooter velocity control and adjustable hood.

**Features:**
- Auto-aim using Limelight (AprilTag detection)
- Fallback to odometry-based targeting
- PIDF velocity control for shooter motors
- Distance-based hood angle adjustment
- Manual control mode

**TODO:**
- Integrate Limelight API for target detection
- Tune turret PID values (`TURRET_KP`, `TURRET_KI`, `TURRET_KD`)
- Tune shooter PIDF values (`SHOOTER_KP`, `SHOOTER_KI`, `SHOOTER_KD`, `SHOOTER_KF`)
- Create hood angle lookup table for different distances
- Implement odometry fallback targeting

### Sorter
Intake and color-based ball sorting system.

**Features:**
- Intake motor control
- Color detection using multiple sensors
- Automatic ball sorting with lopata servos
- Configurable target alliance color
- Manual servo control

**TODO:**
- Calibrate color thresholds (`RED_THRESHOLD`, `BLUE_THRESHOLD`)
- Tune lopata servo positions (`LOPATA_RETRACTED`, `LOPATA_DEPLOYED`)
- Adjust servo deployment duration
- Set alliance color based on configuration

## TeleOp Controls

### Controller 1 (Driver)
- **Left Stick:** Forward/Strafe movement
- **Right Stick X:** Rotation
- **Left Bumper:** Slow mode (50% speed)
- **Right Bumper:** Fast mode (100% speed)
- **A Button:** Toggle field-centric mode
- **B Button:** Reset field-centric heading

### Controller 2 (Operator)
- **Right Trigger:** Run intake
- **Left Trigger:** Run outtake
- **Left Stick Y:** Manual turret control
- **Right Stick Y:** Manual hood control
- **A Button:** Toggle auto-aim mode
- **B Button:** Toggle shooter on/off
- **X Button:** Manually deploy lopata servo 1
- **Y Button:** Manually retract all lopatas
- **DPad Up/Down:** Adjust shooter velocity

## Configuration and Tuning

### Hardware Names
Update hardware device names in `RobotHardware.java` to match your robot configuration file.

### PID Tuning
Tune PID constants in each subsystem:
- **SwerveDrive:** Steering PID (`STEERING_KP`, `STEERING_KI`, `STEERING_KD`)
- **Turret:** Turret position PID and shooter velocity PIDF
- Recommended approach: Start with P-only control, then add D, then I if needed

### Encoder Calibration
- Calibrate steering encoder offsets for each swerve module
- Calibrate turret encoder offset
- Test encoder voltage ranges and update conversion formulas

### Servo Positions
- Test and record optimal servo positions for lopatas
- Test hood servo positions at different shooting distances
- Create lookup table for hood angles

### Alliance Color
Set the alliance color in `TeleOpMain.init()`:
```java
sorter.setAllianceColor(false); // false = blue, true = red
```

## Odometry Integration

The `RobotHardware` class includes placeholders for odometry sensors. Uncomment and configure based on your system:

```java
// For GoBilda Pinpoint:
public GoBildaPinpointDriver odometry;

// For encoder-based odometry:
public DcMotorEx odometryEncoderX;
public DcMotorEx odometryEncoderY;
```

## Limelight Integration

To integrate Limelight for auto-aim:
1. Add Limelight library to your project
2. Initialize Limelight in `TeleOpMain.init()`
3. Update `limelightHasTarget`, `limelightTargetBearing`, and `limelightTargetDistance` in the loop
4. Implement odometry fallback in `Turret.autoAim()`

## Building and Deployment

1. Open the project in Android Studio
2. Connect to your Control Hub via WiFi
3. Build and deploy the OpMode
4. Select "TeleOp Main" from the Driver Station

## Testing Checklist

- [ ] Verify all hardware devices are properly initialized
- [ ] Test swerve drive modules (all 4 modules should drive and steer)
- [ ] Calibrate steering encoder offsets
- [ ] Test field-centric mode and heading reset
- [ ] Test turret rotation and limits
- [ ] Test shooter motors and velocity control
- [ ] Test hood servo movement
- [ ] Test intake motors
- [ ] Calibrate color sensors and test sorting logic
- [ ] Test lopata servo deployment and retraction
- [ ] Tune all PID values for smooth operation
- [ ] Test auto-aim functionality (with Limelight)
- [ ] Verify all controller mappings

## Additional Notes

- All TODO comments in the code indicate areas that require configuration or tuning
- Start with basic functionality and gradually add complexity
- Test each subsystem independently before integrating
- Keep PID tuning values conservative initially and increase gradually
- Monitor telemetry during testing to identify issues

## Support

For FTC programming support:
- [FTC Documentation](https://ftc-docs.firstinspires.org/)
- [FTC Discord](https://discord.gg/first-tech-challenge)
- [FTC Community Forums](https://ftc-community.firstinspires.org/)
