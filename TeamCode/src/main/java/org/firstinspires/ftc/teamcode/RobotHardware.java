package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * RobotHardware class to initialize and manage all hardware devices.
 * This class provides a centralized location for hardware configuration.
 */
public class RobotHardware {
    
    // ========== SWERVE DRIVETRAIN (Control Hub) ==========
    // Drive Motors
    public DcMotorEx frontLeftDrive;
    public DcMotorEx frontRightDrive;
    public DcMotorEx backLeftDrive;
    public DcMotorEx backRightDrive;
    
    // Steering Servos (Continuous Rotation)
    public CRServo frontLeftSteer;
    public CRServo frontRightSteer;
    public CRServo backLeftSteer;
    public CRServo backRightSteer;
    
    // Swerve Encoders (Rev Through Bore - Analog/Absolute mode)
    public AnalogInput frontLeftEncoder;
    public AnalogInput frontRightEncoder;
    public AnalogInput backLeftEncoder;
    public AnalogInput backRightEncoder;
    
    // ========== TURRET & SHOOTER (Expansion Hub) ==========
    public DcMotorEx shooterMotor1;
    public DcMotorEx shooterMotor2;
    public Servo turretServo1;
    public Servo turretServo2;
    public Servo hoodServo;
    public DcMotorEx turretEncoder; // Using motor port as encoder
    
    // ========== INTAKE & SORTING (Expansion Hub) ==========
    public DcMotorEx intakeMotor1;
    public DcMotorEx intakeMotor2;
    
    // Color Sensors (both hubs)
    public ColorSensor colorSensor1;
    public ColorSensor colorSensor2;
    public ColorSensor colorSensor3;
    
    // Lopata (Shovel) Servos
    public Servo lopataServo1;
    public Servo lopataServo2;
    public Servo lopataServo3;
    
    // ========== ODOMETRY ==========
    // GoBilda Pinpoint / Swyft Odometry sensors would be initialized here
    // These typically connect via I2C or as motor encoders
    public DcMotorEx odometryX;
    public DcMotorEx odometryY;
    
    // Hardware Map reference
    private HardwareMap hwMap;
    
    /**
     * Initialize all hardware devices
     * @param hardwareMap The hardware map from the OpMode
     */
    public void init(HardwareMap hardwareMap) {
        hwMap = hardwareMap;
        
        // ========== Initialize Swerve Drivetrain ==========
        // TODO: Replace these hardware names with actual names from Robot Configuration
        frontLeftDrive = hwMap.get(DcMotorEx.class, "frontLeftDrive");
        frontRightDrive = hwMap.get(DcMotorEx.class, "frontRightDrive");
        backLeftDrive = hwMap.get(DcMotorEx.class, "backLeftDrive");
        backRightDrive = hwMap.get(DcMotorEx.class, "backRightDrive");
        
        // Set motor directions (adjust based on robot orientation)
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        
        // Set zero power behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize Steering Servos
        frontLeftSteer = hwMap.get(CRServo.class, "frontLeftSteer");
        frontRightSteer = hwMap.get(CRServo.class, "frontRightSteer");
        backLeftSteer = hwMap.get(CRServo.class, "backLeftSteer");
        backRightSteer = hwMap.get(CRServo.class, "backRightSteer");
        
        // Initialize Swerve Encoders (Analog - Rev Through Bore)
        frontLeftEncoder = hwMap.get(AnalogInput.class, "frontLeftEncoder");
        frontRightEncoder = hwMap.get(AnalogInput.class, "frontRightEncoder");
        backLeftEncoder = hwMap.get(AnalogInput.class, "backLeftEncoder");
        backRightEncoder = hwMap.get(AnalogInput.class, "backRightEncoder");
        
        // ========== Initialize Turret & Shooter ==========
        shooterMotor1 = hwMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hwMap.get(DcMotorEx.class, "shooterMotor2");
        
        shooterMotor1.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotor.Direction.FORWARD);
        
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        turretServo1 = hwMap.get(Servo.class, "turretServo1");
        turretServo2 = hwMap.get(Servo.class, "turretServo2");
        hoodServo = hwMap.get(Servo.class, "hoodServo");
        
        turretEncoder = hwMap.get(DcMotorEx.class, "turretEncoder");
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // ========== Initialize Intake & Sorting ==========
        intakeMotor1 = hwMap.get(DcMotorEx.class, "intakeMotor1");
        intakeMotor2 = hwMap.get(DcMotorEx.class, "intakeMotor2");
        
        intakeMotor1.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor2.setDirection(DcMotor.Direction.FORWARD);
        
        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize Color Sensors
        colorSensor1 = hwMap.get(ColorSensor.class, "colorSensor1");
        colorSensor2 = hwMap.get(ColorSensor.class, "colorSensor2");
        colorSensor3 = hwMap.get(ColorSensor.class, "colorSensor3");
        
        // Initialize Lopata Servos
        lopataServo1 = hwMap.get(Servo.class, "lopataServo1");
        lopataServo2 = hwMap.get(Servo.class, "lopataServo2");
        lopataServo3 = hwMap.get(Servo.class, "lopataServo3");
        
        // Set initial servo positions
        lopataServo1.setPosition(0.5);
        lopataServo2.setPosition(0.5);
        lopataServo3.setPosition(0.5);
        
        // ========== Initialize Odometry ==========
        // Note: Actual implementation depends on specific odometry hardware
        // GoBilda Pinpoint may use I2C or custom driver
        odometryX = hwMap.get(DcMotorEx.class, "odometryX");
        odometryY = hwMap.get(DcMotorEx.class, "odometryY");
        
        odometryX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometryY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometryX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odometryY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
