package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * SwerveModule represents a single swerve drive module with:
 * - Drive motor for propulsion
 * - Steering servo for rotation
 * - Analog encoder (Rev Through Bore) for absolute position
 */
public class SwerveModule {
    
    private final DcMotorEx driveMotor;
    private final CRServo steeringServo;
    private final AnalogInput absoluteEncoder;
    
    // PID Constants for steering control
    // TODO: Tune these values for your specific robot
    private double kP = 0.01;
    private double kI = 0.0;
    private double kD = 0.0;
    
    // Encoder offset to calibrate zero position
    // TODO: Calibrate this value for each module
    private double encoderOffset = 0.0;
    
    // Max voltage from Rev Through Bore Analog encoder
    private static final double MAX_VOLTAGE = 3.3;
    
    // Integrated error and previous error for PID
    private double integratedError = 0.0;
    private double previousError = 0.0;
    
    /**
     * Constructor for SwerveModule
     * @param driveMotor Drive motor for the module
     * @param steeringServo Continuous rotation servo for steering
     * @param absoluteEncoder Analog input for Rev Through Bore encoder
     * @param encoderOffset Offset to calibrate zero position (in radians)
     */
    public SwerveModule(DcMotorEx driveMotor, CRServo steeringServo, 
                       AnalogInput absoluteEncoder, double encoderOffset) {
        this.driveMotor = driveMotor;
        this.steeringServo = steeringServo;
        this.absoluteEncoder = absoluteEncoder;
        this.encoderOffset = encoderOffset;
        
        // Configure drive motor
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    /**
     * Get the current absolute position from the analog encoder
     * @return Current angle in radians (0 to 2*PI)
     */
    public double getAbsolutePosition() {
        // Read voltage from analog encoder
        double voltage = absoluteEncoder.getVoltage();
        
        // Convert voltage to angle (0 to 2*PI)
        double angle = (voltage / MAX_VOLTAGE) * 2 * Math.PI;
        
        // Apply offset
        angle -= encoderOffset;
        
        // Normalize to 0 to 2*PI
        while (angle < 0) angle += 2 * Math.PI;
        while (angle >= 2 * Math.PI) angle -= 2 * Math.PI;
        
        return angle;
    }
    
    /**
     * Get the raw voltage from the encoder
     * @return Raw voltage reading
     */
    public double getRawVoltage() {
        return absoluteEncoder.getVoltage();
    }
    
    /**
     * Set the target angle and drive power for the module
     * @param targetAngle Target angle in radians
     * @param drivePower Drive power (-1.0 to 1.0)
     */
    public void setTargetState(double targetAngle, double drivePower) {
        // Get current position
        double currentAngle = getAbsolutePosition();
        
        // Calculate error
        double error = targetAngle - currentAngle;
        
        // Optimize: take shortest path to target angle
        // If error > PI, go the other way
        if (error > Math.PI) {
            error -= 2 * Math.PI;
        } else if (error < -Math.PI) {
            error += 2 * Math.PI;
        }
        
        // Optimize: if error > 90 degrees, reverse drive direction
        // and subtract 180 degrees from target
        if (Math.abs(error) > Math.PI / 2) {
            error -= Math.signum(error) * Math.PI;
            drivePower = -drivePower;
        }
        
        // PID control for steering
        integratedError += error;
        double derivative = error - previousError;
        previousError = error;
        
        double steeringPower = kP * error + kI * integratedError + kD * derivative;
        
        // Clamp steering power
        steeringPower = Math.max(-1.0, Math.min(1.0, steeringPower));
        
        // Set motor and servo powers
        driveMotor.setPower(drivePower);
        steeringServo.setPower(steeringPower);
    }
    
    /**
     * Stop the module
     */
    public void stop() {
        driveMotor.setPower(0);
        steeringServo.setPower(0);
    }
    
    /**
     * Set PID constants for steering control
     * @param kP Proportional constant
     * @param kI Integral constant
     * @param kD Derivative constant
     */
    public void setPIDConstants(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
    
    /**
     * Reset PID controller
     */
    public void resetPID() {
        integratedError = 0.0;
        previousError = 0.0;
    }
    
    /**
     * Set encoder offset for calibration
     * @param offset Offset in radians
     */
    public void setEncoderOffset(double offset) {
        this.encoderOffset = offset;
    }
}
