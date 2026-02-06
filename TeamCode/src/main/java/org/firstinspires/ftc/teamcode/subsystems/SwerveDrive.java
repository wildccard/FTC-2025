package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * SwerveDrive subsystem handles field-centric swerve drivetrain control.
 * Each swerve module consists of a drive motor, steering servo, and absolute encoder.
 */
public class SwerveDrive {
    
    private RobotHardware hardware;
    
    // Swerve module components
    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;
    
    // Field-centric offset
    private double fieldCentricOffset = 0;
    
    // PID constants for steering control
    // TODO: Tune these values for your robot
    private static final double STEERING_KP = 0.01;
    private static final double STEERING_KI = 0.0;
    private static final double STEERING_KD = 0.0;
    
    // Wheel base dimensions (distance between wheels)
    // TODO: Measure and update these values in inches
    private static final double WHEEL_BASE_WIDTH = 12.0;  // inches
    private static final double WHEEL_BASE_LENGTH = 12.0; // inches
    
    /**
     * Inner class representing a single swerve module
     */
    private class SwerveModule {
        DcMotorEx driveMotor;
        CRServo steeringServo;
        AnalogInput encoder;
        
        double targetAngle = 0;
        double encoderOffset = 0; // TODO: Calibrate encoder offsets
        
        // PID state
        double lastError = 0;
        double integral = 0;
        
        SwerveModule(DcMotorEx drive, CRServo steering, AnalogInput enc) {
            this.driveMotor = drive;
            this.steeringServo = steering;
            this.encoder = enc;
        }
        
        /**
         * Get current angle of the module from the absolute encoder
         * @return Angle in degrees (0-360)
         */
        double getCurrentAngle() {
            double voltage = encoder.getVoltage();
            // Convert voltage to angle (0-360 degrees)
            // TODO: Adjust conversion based on your encoder specifications
            double angle = (voltage / encoder.getMaxVoltage()) * 360.0;
            angle = (angle - encoderOffset + 360) % 360;
            return angle;
        }
        
        /**
         * Set the target angle and drive power for this module
         */
        void set(double angle, double power) {
            targetAngle = angle;
            
            // Calculate shortest path to target angle
            double currentAngle = getCurrentAngle();
            double error = angleDifference(targetAngle, currentAngle);
            
            // Optimize by reversing drive direction if needed
            if (Math.abs(error) > 90) {
                error = angleDifference(targetAngle + 180, currentAngle);
                power = -power;
            }
            
            // PID control for steering
            integral += error;
            double derivative = error - lastError;
            double steeringPower = STEERING_KP * error + STEERING_KI * integral + STEERING_KD * derivative;
            lastError = error;
            
            // Clamp steering power
            steeringPower = Math.max(-1.0, Math.min(1.0, steeringPower));
            
            // Set powers
            steeringServo.setPower(steeringPower);
            driveMotor.setPower(power);
        }
        
        /**
         * Stop the module
         */
        void stop() {
            driveMotor.setPower(0);
            steeringServo.setPower(0);
        }
    }
    
    /**
     * Constructor
     */
    public SwerveDrive(RobotHardware hw) {
        this.hardware = hw;
        
        // Initialize swerve modules
        frontLeft = new SwerveModule(
            hardware.driveMotorFL,
            hardware.steeringServoFL,
            hardware.steeringEncoderFL
        );
        
        frontRight = new SwerveModule(
            hardware.driveMotorFR,
            hardware.steeringServoFR,
            hardware.steeringEncoderFR
        );
        
        backLeft = new SwerveModule(
            hardware.driveMotorBL,
            hardware.steeringServoBL,
            hardware.steeringEncoderBL
        );
        
        backRight = new SwerveModule(
            hardware.driveMotorBR,
            hardware.steeringServoBR,
            hardware.steeringEncoderBR
        );
    }
    
    /**
     * Drive the robot using field-centric control
     * @param forward Forward/backward movement (-1 to 1)
     * @param strafe Left/right movement (-1 to 1)
     * @param rotate Rotation (-1 to 1)
     * @param fieldCentric Whether to use field-centric control
     */
    public void drive(double forward, double strafe, double rotate, boolean fieldCentric) {
        // Get current heading for field-centric control
        double heading = 0;
        if (fieldCentric) {
            heading = Math.toRadians(hardware.getHeading() - fieldCentricOffset);
            
            // Rotate input vectors by heading
            double temp = forward * Math.cos(heading) + strafe * Math.sin(heading);
            strafe = -forward * Math.sin(heading) + strafe * Math.cos(heading);
            forward = temp;
        }
        
        // Calculate wheel vectors
        // Using standard swerve kinematics
        double r = Math.sqrt(WHEEL_BASE_WIDTH * WHEEL_BASE_WIDTH + WHEEL_BASE_LENGTH * WHEEL_BASE_LENGTH);
        
        double a = strafe - rotate * (WHEEL_BASE_LENGTH / r);
        double b = strafe + rotate * (WHEEL_BASE_LENGTH / r);
        double c = forward - rotate * (WHEEL_BASE_WIDTH / r);
        double d = forward + rotate * (WHEEL_BASE_WIDTH / r);
        
        // Calculate wheel speeds and angles
        double flSpeed = Math.sqrt(b * b + c * c);
        double frSpeed = Math.sqrt(b * b + d * d);
        double blSpeed = Math.sqrt(a * a + c * c);
        double brSpeed = Math.sqrt(a * a + d * d);
        
        double flAngle = Math.toDegrees(Math.atan2(b, c));
        double frAngle = Math.toDegrees(Math.atan2(b, d));
        double blAngle = Math.toDegrees(Math.atan2(a, c));
        double brAngle = Math.toDegrees(Math.atan2(a, d));
        
        // Normalize wheel speeds
        double max = Math.max(Math.max(flSpeed, frSpeed), Math.max(blSpeed, brSpeed));
        if (max > 1.0) {
            flSpeed /= max;
            frSpeed /= max;
            blSpeed /= max;
            brSpeed /= max;
        }
        
        // Set module states
        frontLeft.set(flAngle, flSpeed);
        frontRight.set(frAngle, frSpeed);
        backLeft.set(blAngle, blSpeed);
        backRight.set(brAngle, brSpeed);
    }
    
    /**
     * Stop all swerve modules
     */
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
    
    /**
     * Reset field-centric offset to current heading
     */
    public void resetFieldCentric() {
        fieldCentricOffset = hardware.getHeading();
    }
    
    /**
     * Calculate the difference between two angles, accounting for wrap-around
     */
    private double angleDifference(double target, double current) {
        double diff = target - current;
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;
        return diff;
    }
    
    /**
     * Get telemetry data for debugging
     */
    public String getTelemetry() {
        return String.format("FL: %.1f° FR: %.1f° BL: %.1f° BR: %.1f°",
            frontLeft.getCurrentAngle(),
            frontRight.getCurrentAngle(),
            backLeft.getCurrentAngle(),
            backRight.getCurrentAngle());
    }
}
