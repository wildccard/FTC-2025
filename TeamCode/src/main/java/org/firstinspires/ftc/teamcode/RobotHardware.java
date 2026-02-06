package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * RobotHardware initializes and provides access to all hardware devices on the robot.
 * This includes motors, servos, sensors, and odometry devices.
 */
public class RobotHardware {
    
    // ========== SWERVE DRIVETRAIN (Control Hub) ==========
    // 4x Drivetrain Motors
    public DcMotorEx driveMotorFL;  // Front Left
    public DcMotorEx driveMotorFR;  // Front Right
    public DcMotorEx driveMotorBL;  // Back Left
    public DcMotorEx driveMotorBR;  // Back Right
    
    // 4x Steering Servos (Continuous Rotation)
    public CRServo steeringServoFL;
    public CRServo steeringServoFR;
    public CRServo steeringServoBL;
    public CRServo steeringServoBR;
    
    // 4x Steering Encoders (Analog/Absolute)
    public AnalogInput steeringEncoderFL;
    public AnalogInput steeringEncoderFR;
    public AnalogInput steeringEncoderBL;
    public AnalogInput steeringEncoderBR;
    
    // IMU for field-centric driving
    public IMU imu;
    
    // ========== TURRET & SHOOTER (Expansion Hub) ==========
    // 2x Shooter Motors
    public DcMotorEx shooterMotor1;
    public DcMotorEx shooterMotor2;
    
    // 2x Turret Servos
    public Servo turretServo1;
    public Servo turretServo2;
    
    // 1x Hood Servo
    public Servo hoodServo;
    
    // 1x Turret Encoder
    public AnalogInput turretEncoder;
    
    // ========== INTAKE & SORTING SYSTEM (Expansion Hub) ==========
    // 2x Intake Motors
    public DcMotorEx intakeMotor1;
    public DcMotorEx intakeMotor2;
    
    // Color Sensors for detecting ball color
    public ColorSensor colorSensor1;
    public ColorSensor colorSensor2;
    public ColorSensor colorSensor3;
    
    // 3x "Lopata" (Shovel) Servos for sorting/diverting balls
    public Servo lopataServo1;
    public Servo lopataServo2;
    public Servo lopataServo3;
    
    // ========== ODOMETRY ==========
    // GoBilda Pinpoint / Swyft Odometry (X & Y) sensors
    // TODO: Add specific odometry sensor initialization based on which system is used
    // public GoBildaPinpointDriver odometry; // Uncomment if using GoBilda Pinpoint
    // OR
    // public DcMotorEx odometryEncoderX;
    // public DcMotorEx odometryEncoderY;
    
    // Hardware map reference
    private HardwareMap hwMap = null;
    
    /**
     * Initialize all hardware devices
     * @param ahwMap The hardware map from the OpMode
     */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        
        // ========== Initialize Swerve Drivetrain ==========
        // TODO: Replace placeholder names with actual hardware configuration names
        driveMotorFL = hwMap.get(DcMotorEx.class, "driveFL");
        driveMotorFR = hwMap.get(DcMotorEx.class, "driveFR");
        driveMotorBL = hwMap.get(DcMotorEx.class, "driveBL");
        driveMotorBR = hwMap.get(DcMotorEx.class, "driveBR");
        
        // Set motor directions (adjust based on robot orientation)
        driveMotorFL.setDirection(DcMotor.Direction.FORWARD);
        driveMotorFR.setDirection(DcMotor.Direction.REVERSE);
        driveMotorBL.setDirection(DcMotor.Direction.FORWARD);
        driveMotorBR.setDirection(DcMotor.Direction.REVERSE);
        
        // Set zero power behavior
        driveMotorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveMotorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveMotorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveMotorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize steering servos
        steeringServoFL = hwMap.get(CRServo.class, "steerFL");
        steeringServoFR = hwMap.get(CRServo.class, "steerFR");
        steeringServoBL = hwMap.get(CRServo.class, "steerBL");
        steeringServoBR = hwMap.get(CRServo.class, "steerBR");
        
        // Initialize steering encoders
        steeringEncoderFL = hwMap.get(AnalogInput.class, "encoderFL");
        steeringEncoderFR = hwMap.get(AnalogInput.class, "encoderFR");
        steeringEncoderBL = hwMap.get(AnalogInput.class, "encoderBL");
        steeringEncoderBR = hwMap.get(AnalogInput.class, "encoderBR");
        
        // Initialize IMU for field-centric driving
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(imuParameters);
        
        // ========== Initialize Turret & Shooter ==========
        shooterMotor1 = hwMap.get(DcMotorEx.class, "shooter1");
        shooterMotor2 = hwMap.get(DcMotorEx.class, "shooter2");
        
        shooterMotor1.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotor.Direction.FORWARD);
        
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        // Set shooter motors to velocity control mode
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        turretServo1 = hwMap.get(Servo.class, "turret1");
        turretServo2 = hwMap.get(Servo.class, "turret2");
        hoodServo = hwMap.get(Servo.class, "hood");
        
        turretEncoder = hwMap.get(AnalogInput.class, "turretEncoder");
        
        // ========== Initialize Intake & Sorting System ==========
        intakeMotor1 = hwMap.get(DcMotorEx.class, "intake1");
        intakeMotor2 = hwMap.get(DcMotorEx.class, "intake2");
        
        intakeMotor1.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor2.setDirection(DcMotor.Direction.FORWARD);
        
        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize color sensors
        colorSensor1 = hwMap.get(ColorSensor.class, "colorSensor1");
        colorSensor2 = hwMap.get(ColorSensor.class, "colorSensor2");
        colorSensor3 = hwMap.get(ColorSensor.class, "colorSensor3");
        
        // Initialize lopata (shovel) servos
        lopataServo1 = hwMap.get(Servo.class, "lopata1");
        lopataServo2 = hwMap.get(Servo.class, "lopata2");
        lopataServo3 = hwMap.get(Servo.class, "lopata3");
        
        // Set initial servo positions
        lopataServo1.setPosition(0.5); // TODO: Tune servo positions
        lopataServo2.setPosition(0.5);
        lopataServo3.setPosition(0.5);
        
        // ========== Initialize Odometry ==========
        // TODO: Initialize specific odometry system
        // Example for encoder-based odometry:
        // odometryEncoderX = hwMap.get(DcMotorEx.class, "odoX");
        // odometryEncoderY = hwMap.get(DcMotorEx.class, "odoY");
        // odometryEncoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // odometryEncoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // odometryEncoderX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // odometryEncoderY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    /**
     * Reset the IMU yaw angle to zero
     */
    public void resetIMU() {
        imu.resetYaw();
    }
    
    /**
     * Get the robot's current heading from the IMU
     * @return The robot's heading in degrees
     */
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
