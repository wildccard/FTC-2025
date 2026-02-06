# FTC 2025 Robot Code - TeamCode

This module contains the robot code for the 2025 FTC season swerve robot with turret and intake system.

## Project Structure

```
TeamCode/
├── src/main/java/org/firstinspires/ftc/teamcode/
│   ├── RobotHardware.java           # Hardware initialization and configuration
│   ├── subsystems/
│   │   ├── SwerveModule.java        # Individual swerve module with analog encoder
│   │   ├── SwerveDrive.java         # Field-centric swerve drive subsystem
│   │   ├── Turret.java              # Turret with auto-aim and shooter
│   │   └── Sorter.java              # Intake and color-based sorting
│   └── opmodes/
│       └── TeleOpMain.java          # Main TeleOp OpMode
└── build.gradle                      # Build configuration
```

## Robot Configuration

### 1. Swerve Drivetrain (Control Hub)
- **4x Drive Motors:** `frontLeftDrive`, `frontRightDrive`, `backLeftDrive`, `backRightDrive`
- **4x Steering Servos (CR):** `frontLeftSteer`, `frontRightSteer`, `backLeftSteer`, `backRightSteer`
- **4x Analog Encoders:** `frontLeftEncoder`, `frontRightEncoder`, `backLeftEncoder`, `backRightEncoder`
  - **Type:** Rev Through Bore Encoder V1 (Analog/Absolute mode)
  - **Connection:** Analog Input ports on Control Hub

### 2. Turret & Shooter (Expansion Hub)
- **2x Shooter Motors:** `shooterMotor1`, `shooterMotor2`
- **2x Turret Servos:** `turretServo1`, `turretServo2`
- **1x Hood Servo:** `hoodServo`
- **1x Turret Encoder:** `turretEncoder` (motor port used as encoder)

### 3. Intake & Sorting (Expansion Hub)
- **2x Intake Motors:** `intakeMotor1`, `intakeMotor2`
- **3x Color Sensors:** `colorSensor1`, `colorSensor2`, `colorSensor3`
- **3x Lopata Servos:** `lopataServo1`, `lopataServo2`, `lopataServo3`

### 4. Odometry
- **Odometry Sensors:** `odometryX`, `odometryY` (GoBilda Pinpoint / Swyft)

## Hardware Configuration Steps

1. **Configure devices in the Robot Controller app** using the names listed above
2. **Calibrate swerve encoder offsets** in `SwerveDrive.java` constructor
3. **Tune PID constants** for steering and turret control
4. **Calibrate color sensor thresholds** in `Sorter.java`
5. **Set lopata servo positions** in `Sorter.java`
6. **Configure hood angle mapping** based on distance testing

## TeleOp Controls

### Gamepad 1 (Driver)
- **Left Stick:** Strafe (X/Y translation)
- **Right Stick X:** Rotation
- **A Button:** Reset field-centric heading
- **B Button:** Toggle speed mode (slow/normal)

### Gamepad 2 (Operator)
- **Right Bumper:** Start intake
- **Left Bumper:** Reverse intake
- **A Button:** Toggle auto-aim and shooter
- **B Button:** Stop shooter
- **X Button:** Manual turret left
- **Y Button:** Manual turret right
- **D-pad Up:** Increase hood angle
- **D-pad Down:** Decrease hood angle
- **Right Trigger:** Manual shooter speed control

## Key Features

### Swerve Drive
- **Field-centric control** using IMU/gyro readings
- **Analog absolute encoders** (Rev Through Bore) for module positioning
- **PID-controlled steering** for precise module angles
- **Optimized module states** to minimize rotation

### Turret System
- **Auto-aim using Limelight** (AprilTag detection)
- **Odometry fallback** when target is lost
- **Distance-based hood adjustment** for optimal shooting angle
- **PID-controlled turret positioning**

### Sorting System
- **Color sensor detection** of ball color (red/blue)
- **Automatic sorting** with lopata (shovel) servos
- **Configurable target color** based on alliance
- **Sequential ball launching** (NEW!):
  - Launch balls upward in order specified by AprilTag
  - AprilTag ID decoded at match start
  - Precise timing control for each launch
  - D-pad controls for starting/stopping sequence

## TODO: Calibration and Tuning

The following values need to be calibrated for your specific robot:

### In `SwerveModule.java`:
- [ ] `kP`, `kI`, `kD` - Steering PID constants (line 23-25)
- [ ] `encoderOffset` - Zero position offset for each module (line 28)

### In `SwerveDrive.java`:
- [ ] `TRACK_WIDTH` - Distance between left/right wheels (line 15)
- [ ] `WHEEL_BASE` - Distance between front/back wheels (line 16)
- [ ] Encoder offsets for each module (lines 38, 45, 52, 59)
- [ ] PID constants (line 65)

### In `Turret.java`:
- [ ] `turretKP`, `turretKI`, `turretKD` - Turret aiming PID (lines 21-23)
- [ ] `MIN_SHOOTER_SPEED`, `MAX_SHOOTER_SPEED` - Shooter speed range (lines 26-27)
- [ ] Hood angle mapping (lines 30-34)
- [ ] Target position for odometry fallback (lines 48-49)

### In `Sorter.java`:
- [ ] `INTAKE_SPEED` - Intake motor speed (line 23)
- [ ] `RED_THRESHOLD`, `BLUE_THRESHOLD` - Color detection thresholds (lines 26-28)
- [ ] `LOPATA_NEUTRAL`, `LOPATA_SORT_LEFT`, `LOPATA_SORT_RIGHT` - Servo positions (lines 31-33)

## Integration Notes

### Limelight Integration
To integrate Limelight for auto-aim:
```java
// In TeleOpMain.loop()
// Get Limelight data (pseudo-code)
double bearing = limelight.getTargetBearing();
double distance = limelight.getTargetDistance();
boolean hasTarget = limelight.hasTarget();
turret.updateLimelightTarget(bearing, distance, hasTarget);
```

### Odometry Integration
To integrate odometry for field positioning:
```java
// In TeleOpMain.loop()
// Get odometry data
double x = odometry.getX();
double y = odometry.getY();
double heading = odometry.getHeading();
turret.updateRobotPosition(x, y, heading);
swerveDrive.updateHeading(heading);
```

### IMU Integration
For field-centric swerve drive:
```java
// In TeleOpMain.loop()
// Get IMU heading
double heading = imu.getHeading();
swerveDrive.updateHeading(heading);
```

## Building and Deployment

This code is designed to work within the standard FTC SDK project structure. To use:

1. Copy the `TeamCode` directory to your FTC SDK project
2. Configure all hardware devices in the Robot Controller app
3. Build and deploy using Android Studio or the FTC SDK build system
4. Select "TeleOpMain" from the TeleOp menu on the Driver Station

## Notes

- All hardware device names in `RobotHardware.java` must match the configuration in the Robot Controller
- The code includes extensive telemetry for debugging during testing
- Color sensor calibration is critical for reliable sorting
- Swerve module encoder offsets must be calibrated after assembly

## Support

For questions or issues, please refer to the FTC SDK documentation and community resources.
