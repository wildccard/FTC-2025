# Quick Reference Guide - FTC 2025 TeleOp

## File Locations

### Main Files
- **RobotHardware**: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotHardware.java`
- **TeleOpMain**: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/TeleOpMain.java`

### Subsystems
- **SwerveModule**: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/SwerveModule.java`
- **SwerveDrive**: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/SwerveDrive.java`
- **Turret**: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Turret.java`
- **Sorter**: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Sorter.java`

## Hardware Device Names (Configure in Robot Controller)

### Control Hub
```
frontLeftDrive      - DcMotorEx
frontRightDrive     - DcMotorEx
backLeftDrive       - DcMotorEx
backRightDrive      - DcMotorEx
frontLeftSteer      - CRServo
frontRightSteer     - CRServo
backLeftSteer       - CRServo
backRightSteer      - CRServo
frontLeftEncoder    - AnalogInput (Rev Through Bore)
frontRightEncoder   - AnalogInput (Rev Through Bore)
backLeftEncoder     - AnalogInput (Rev Through Bore)
backRightEncoder    - AnalogInput (Rev Through Bore)
```

### Expansion Hub
```
shooterMotor1       - DcMotorEx
shooterMotor2       - DcMotorEx
turretServo1        - Servo
turretServo2        - Servo
hoodServo           - Servo
turretEncoder       - DcMotorEx (used as encoder)
intakeMotor1        - DcMotorEx
intakeMotor2        - DcMotorEx
colorSensor1        - ColorSensor
colorSensor2        - ColorSensor
colorSensor3        - ColorSensor
lopataServo1        - Servo
lopataServo2        - Servo
lopataServo3        - Servo
odometryX           - DcMotorEx (odometry pod)
odometryY           - DcMotorEx (odometry pod)
```

## TeleOp Controls

### Gamepad 1 (Driver)
| Control | Function |
|---------|----------|
| Left Stick | Strafe (X/Y translation) |
| Right Stick X | Rotation |
| A Button | Reset field-centric heading |
| B Button | Toggle speed mode (slow/normal) |

### Gamepad 2 (Operator)
| Control | Function |
|---------|----------|
| Right Bumper | Start intake |
| Left Bumper | Reverse intake |
| A Button | Toggle auto-aim and shooter |
| B Button | Stop shooter |
| X Button | Manual turret left |
| Y Button | Manual turret right |
| D-pad Up | Increase hood angle |
| D-pad Down | Decrease hood angle |
| Right Trigger | Manual shooter speed |

## Calibration Checklist

### Before First Test
- [ ] Configure all hardware devices in Robot Controller app
- [ ] Verify motor directions in RobotHardware.java
- [ ] Set alliance color in TeleOpMain.java (line 55)

### After Assembly
- [ ] Calibrate swerve encoder offsets (SwerveDrive.java lines 38, 45, 52, 59)
- [ ] Measure and set robot dimensions (SwerveDrive.java lines 15-16)
- [ ] Test individual swerve modules
- [ ] Verify turret servo directions

### During Testing
- [ ] Tune steering PID (SwerveModule.java lines 22-24)
- [ ] Tune turret PID (Turret.java lines 21-23)
- [ ] Calibrate color sensor thresholds (Sorter.java lines 26-28)
- [ ] Set lopata servo positions (Sorter.java lines 31-33)
- [ ] Test hood angle mapping (Turret.java lines 30-34)

## Important Notes

### Rev Through Bore Encoders
- Connected to **Analog Input** ports
- Voltage range: 0-3.3V
- One full rotation = 0-3.3V
- Absolute position (no reset needed)

### Swerve Drive
- Field-centric by default
- Requires IMU/gyro integration for heading
- Reset heading with gamepad1.A

### Turret
- Auto-aim requires Limelight integration
- Falls back to odometry when target lost
- Hood adjusts automatically based on distance

### Sorter
- Detects red/blue balls
- Automatically sorts based on alliance color
- Can disable auto-sort if needed

## Troubleshooting

### Swerve modules not responding
- Check encoder wiring (analog ports)
- Verify encoder offsets are calibrated
- Check steering servo directions

### Turret not aiming
- Integrate Limelight data in TeleOpMain.loop()
- Verify turret servo directions
- Tune PID constants

### Color sensors not detecting
- Adjust lighting conditions
- Calibrate thresholds in Sorter.java
- Check sensor wiring and I2C addresses

## Next Steps

1. Review [TeamCode/README.md](README.md) for detailed documentation
2. Configure hardware in Robot Controller app
3. Test each subsystem individually
4. Calibrate sensors and PID values
5. Integrate IMU, Limelight, and odometry
6. Run full system test
