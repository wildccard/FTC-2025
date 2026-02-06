package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Sorter subsystem handles intake and color-based ball sorting.
 * Features:
 * - Intake control
 * - Color detection using multiple sensors
 * - Automatic ball sorting using lopata (shovel) servos
 */
public class Sorter {
    
    private RobotHardware hardware;
    
    // Intake state
    private double intakePower = 0;
    
    // Color detection thresholds
    // TODO: Calibrate these values for your specific sensors and game elements
    private static final int RED_THRESHOLD = 200;
    private static final int BLUE_THRESHOLD = 200;
    private static final int GREEN_THRESHOLD = 150;
    
    // Lopata servo positions
    // TODO: Calibrate these positions for your robot
    private static final double LOPATA_RETRACTED = 0.5;  // Neutral position
    private static final double LOPATA_DEPLOYED = 1.0;   // Deployed to divert ball
    
    // Color detection state for each sensor
    private BallColor sensor1DetectedColor = BallColor.NONE;
    private BallColor sensor2DetectedColor = BallColor.NONE;
    private BallColor sensor3DetectedColor = BallColor.NONE;
    
    // Timers for servo actuation
    private ElapsedTime servo1Timer = new ElapsedTime();
    private ElapsedTime servo2Timer = new ElapsedTime();
    private ElapsedTime servo3Timer = new ElapsedTime();
    
    // Servo deployment duration (seconds)
    private static final double SERVO_DEPLOY_DURATION = 0.5;
    
    // Target ball color (which color to accept)
    private BallColor targetColor = BallColor.BLUE; // Default to blue alliance
    
    /**
     * Enum for ball colors
     */
    public enum BallColor {
        NONE,
        RED,
        BLUE,
        UNKNOWN
    }
    
    /**
     * Constructor
     */
    public Sorter(RobotHardware hw) {
        this.hardware = hw;
    }
    
    /**
     * Update the sorter subsystem (call this periodically)
     */
    public void update() {
        // Update intake motors
        hardware.intakeMotor1.setPower(intakePower);
        hardware.intakeMotor2.setPower(intakePower);
        
        // Check each color sensor and activate corresponding lopata servo
        updateColorSensor1();
        updateColorSensor2();
        updateColorSensor3();
        
        // Auto-retract servos after deployment duration
        autoRetractServos();
    }
    
    /**
     * Update color sensor 1 and control lopata servo 1
     */
    private void updateColorSensor1() {
        sensor1DetectedColor = detectColor(hardware.colorSensor1);
        
        if (sensor1DetectedColor != BallColor.NONE && sensor1DetectedColor != targetColor) {
            // Wrong color detected, deploy lopata to divert
            hardware.lopataServo1.setPosition(LOPATA_DEPLOYED);
            servo1Timer.reset();
        }
    }
    
    /**
     * Update color sensor 2 and control lopata servo 2
     */
    private void updateColorSensor2() {
        sensor2DetectedColor = detectColor(hardware.colorSensor2);
        
        if (sensor2DetectedColor != BallColor.NONE && sensor2DetectedColor != targetColor) {
            // Wrong color detected, deploy lopata to divert
            hardware.lopataServo2.setPosition(LOPATA_DEPLOYED);
            servo2Timer.reset();
        }
    }
    
    /**
     * Update color sensor 3 and control lopata servo 3
     */
    private void updateColorSensor3() {
        sensor3DetectedColor = detectColor(hardware.colorSensor3);
        
        if (sensor3DetectedColor != BallColor.NONE && sensor3DetectedColor != targetColor) {
            // Wrong color detected, deploy lopata to divert
            hardware.lopataServo3.setPosition(LOPATA_DEPLOYED);
            servo3Timer.reset();
        }
    }
    
    /**
     * Auto-retract servos after deployment duration
     */
    private void autoRetractServos() {
        if (servo1Timer.seconds() > SERVO_DEPLOY_DURATION) {
            hardware.lopataServo1.setPosition(LOPATA_RETRACTED);
        }
        
        if (servo2Timer.seconds() > SERVO_DEPLOY_DURATION) {
            hardware.lopataServo2.setPosition(LOPATA_RETRACTED);
        }
        
        if (servo3Timer.seconds() > SERVO_DEPLOY_DURATION) {
            hardware.lopataServo3.setPosition(LOPATA_RETRACTED);
        }
    }
    
    /**
     * Detect ball color from a color sensor
     * @param sensor The color sensor to read
     * @return The detected ball color
     */
    private BallColor detectColor(ColorSensor sensor) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();
        
        // Check if any color is detected (above background noise)
        int maxColor = Math.max(Math.max(red, green), blue);
        
        if (maxColor < 50) {
            // No ball detected (all values too low)
            return BallColor.NONE;
        }
        
        // Determine which color is dominant
        if (red > BLUE_THRESHOLD && red > blue * 1.5) {
            // Red ball detected
            return BallColor.RED;
        } else if (blue > RED_THRESHOLD && blue > red * 1.5) {
            // Blue ball detected
            return BallColor.BLUE;
        } else {
            // Unclear color
            return BallColor.UNKNOWN;
        }
    }
    
    /**
     * Set intake power
     * @param power Power level (-1.0 to 1.0)
     */
    public void setIntakePower(double power) {
        this.intakePower = power;
    }
    
    /**
     * Run intake at full power
     */
    public void intake() {
        setIntakePower(1.0);
    }
    
    /**
     * Run intake in reverse (outtake)
     */
    public void outtake() {
        setIntakePower(-1.0);
    }
    
    /**
     * Stop intake
     */
    public void stopIntake() {
        setIntakePower(0);
    }
    
    /**
     * Set the target ball color to accept
     * @param color The target color (RED or BLUE)
     */
    public void setTargetColor(BallColor color) {
        if (color == BallColor.RED || color == BallColor.BLUE) {
            this.targetColor = color;
        }
    }
    
    /**
     * Set target color based on alliance
     * @param isRedAlliance True if on red alliance, false if on blue alliance
     */
    public void setAllianceColor(boolean isRedAlliance) {
        this.targetColor = isRedAlliance ? BallColor.RED : BallColor.BLUE;
    }
    
    /**
     * Manually deploy a specific lopata servo
     * @param servoNumber Servo number (1, 2, or 3)
     */
    public void deployLopata(int servoNumber) {
        switch (servoNumber) {
            case 1:
                hardware.lopataServo1.setPosition(LOPATA_DEPLOYED);
                servo1Timer.reset();
                break;
            case 2:
                hardware.lopataServo2.setPosition(LOPATA_DEPLOYED);
                servo2Timer.reset();
                break;
            case 3:
                hardware.lopataServo3.setPosition(LOPATA_DEPLOYED);
                servo3Timer.reset();
                break;
        }
    }
    
    /**
     * Manually retract a specific lopata servo
     * @param servoNumber Servo number (1, 2, or 3)
     */
    public void retractLopata(int servoNumber) {
        switch (servoNumber) {
            case 1:
                hardware.lopataServo1.setPosition(LOPATA_RETRACTED);
                break;
            case 2:
                hardware.lopataServo2.setPosition(LOPATA_RETRACTED);
                break;
            case 3:
                hardware.lopataServo3.setPosition(LOPATA_RETRACTED);
                break;
        }
    }
    
    /**
     * Retract all lopata servos
     */
    public void retractAllLopatas() {
        hardware.lopataServo1.setPosition(LOPATA_RETRACTED);
        hardware.lopataServo2.setPosition(LOPATA_RETRACTED);
        hardware.lopataServo3.setPosition(LOPATA_RETRACTED);
    }
    
    /**
     * Get the current intake power
     */
    public double getIntakePower() {
        return intakePower;
    }
    
    /**
     * Check if any sensor detects a ball
     */
    public boolean isBallDetected() {
        return sensor1DetectedColor != BallColor.NONE ||
               sensor2DetectedColor != BallColor.NONE ||
               sensor3DetectedColor != BallColor.NONE;
    }
    
    /**
     * Get telemetry data for debugging
     */
    public String getTelemetry() {
        return String.format("Intake: %.2f | S1:%s S2:%s S3:%s | Target:%s",
            intakePower,
            sensor1DetectedColor.name(),
            sensor2DetectedColor.name(),
            sensor3DetectedColor.name(),
            targetColor.name());
    }
    
    /**
     * Get detailed color sensor readings for debugging
     */
    public String getColorSensorTelemetry() {
        return String.format("RGB1: %d,%d,%d | RGB2: %d,%d,%d | RGB3: %d,%d,%d",
            hardware.colorSensor1.red(), hardware.colorSensor1.green(), hardware.colorSensor1.blue(),
            hardware.colorSensor2.red(), hardware.colorSensor2.green(), hardware.colorSensor2.blue(),
            hardware.colorSensor3.red(), hardware.colorSensor3.green(), hardware.colorSensor3.blue());
    }
}
