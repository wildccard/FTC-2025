package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Turret subsystem handles auto-aim, shooter control, and hood adjustment.
 * Features:
 * - Auto-aim using Limelight (AprilTag) for bearing/distance
 * - Fallback to Odometry targeting if target is lost
 * - Adjustable hood angle based on distance
 */
public class Turret {
    
    private RobotHardware hardware;
    
    // Shooter velocity (in ticks per second)
    private double targetShooterVelocity = 0;
    
    // Turret and hood positions
    private double targetTurretAngle = 0;
    private double targetHoodAngle = 0.5;
    
    // Targeting state
    private boolean isAutoAiming = false;
    private double lastKnownTargetBearing = 0;
    private double lastKnownTargetDistance = 0;
    private ElapsedTime targetLostTimer = new ElapsedTime();
    
    // PID constants for turret control
    // TODO: Tune these values for your robot
    private static final double TURRET_KP = 0.01;
    private static final double TURRET_KI = 0.0;
    private static final double TURRET_KD = 0.0;
    
    // Shooter velocity PID constants
    // TODO: Tune these values for your robot
    private static final double SHOOTER_KP = 5.0;
    private static final double SHOOTER_KI = 0.1;
    private static final double SHOOTER_KD = 0.0;
    private static final double SHOOTER_KF = 0.1; // Feedforward
    
    // Turret limits (in degrees)
    private static final double TURRET_MIN_ANGLE = -180;
    private static final double TURRET_MAX_ANGLE = 180;
    
    // Hood servo positions
    private static final double HOOD_MIN_POSITION = 0.0;
    private static final double HOOD_MAX_POSITION = 1.0;
    
    // Target lost timeout (in seconds)
    private static final double TARGET_LOST_TIMEOUT = 2.0;
    
    // PID state
    private double turretLastError = 0;
    private double turretIntegral = 0;
    
    /**
     * Constructor
     */
    public Turret(RobotHardware hw) {
        this.hardware = hw;
    }
    
    /**
     * Update the turret subsystem (call this periodically)
     */
    public void update() {
        // Update turret position control
        updateTurretPosition();
        
        // Update shooter velocity control
        updateShooterVelocity();
        
        // Update hood position
        updateHoodPosition();
    }
    
    /**
     * Update turret position using PID control
     */
    private void updateTurretPosition() {
        double currentAngle = getTurretAngle();
        double error = targetTurretAngle - currentAngle;
        
        // Wrap error to [-180, 180]
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        
        turretIntegral += error;
        double derivative = error - turretLastError;
        double servoPower = TURRET_KP * error + TURRET_KI * turretIntegral + TURRET_KD * derivative;
        turretLastError = error;
        
        // Clamp and apply to servos
        servoPower = Math.max(-1.0, Math.min(1.0, servoPower));
        
        // Both turret servos work together
        hardware.turretServo1.setPosition(0.5 + servoPower * 0.5);
        hardware.turretServo2.setPosition(0.5 + servoPower * 0.5);
    }
    
    /**
     * Update shooter velocity using PIDF control
     */
    private void updateShooterVelocity() {
        if (targetShooterVelocity > 0) {
            // Get current velocities
            double currentVelocity1 = hardware.shooterMotor1.getVelocity();
            double currentVelocity2 = hardware.shooterMotor2.getVelocity();
            double avgVelocity = (currentVelocity1 + currentVelocity2) / 2.0;
            
            // Calculate PIDF output
            double error = targetShooterVelocity - avgVelocity;
            double feedforward = SHOOTER_KF * targetShooterVelocity;
            
            // Simple P controller with feedforward (can be expanded to full PIDF)
            double power = (SHOOTER_KP * error + feedforward) / hardware.shooterMotor1.getMotorType().getMaxRPM();
            power = Math.max(0, Math.min(1.0, power));
            
            hardware.shooterMotor1.setPower(power);
            hardware.shooterMotor2.setPower(power);
        } else {
            hardware.shooterMotor1.setPower(0);
            hardware.shooterMotor2.setPower(0);
        }
    }
    
    /**
     * Update hood position
     */
    private void updateHoodPosition() {
        double clampedPosition = Math.max(HOOD_MIN_POSITION, Math.min(HOOD_MAX_POSITION, targetHoodAngle));
        hardware.hoodServo.setPosition(clampedPosition);
    }
    
    /**
     * Get current turret angle from encoder
     * @return Turret angle in degrees
     */
    public double getTurretAngle() {
        double voltage = hardware.turretEncoder.getVoltage();
        // Convert voltage to angle (-180 to 180 degrees)
        // TODO: Adjust conversion based on your encoder specifications
        double angle = (voltage / hardware.turretEncoder.getMaxVoltage()) * 360.0 - 180.0;
        return angle;
    }
    
    /**
     * Set shooter velocity
     * @param velocityTPS Velocity in ticks per second
     */
    public void setShooterVelocity(double velocityTPS) {
        this.targetShooterVelocity = velocityTPS;
    }
    
    /**
     * Stop the shooter
     */
    public void stopShooter() {
        this.targetShooterVelocity = 0;
    }
    
    /**
     * Set turret angle manually
     * @param angle Target angle in degrees (-180 to 180)
     */
    public void setTurretAngle(double angle) {
        this.targetTurretAngle = Math.max(TURRET_MIN_ANGLE, Math.min(TURRET_MAX_ANGLE, angle));
        this.isAutoAiming = false;
    }
    
    /**
     * Set hood angle manually
     * @param position Servo position (0.0 to 1.0)
     */
    public void setHoodAngle(double position) {
        this.targetHoodAngle = position;
    }
    
    /**
     * Enable auto-aim mode with Limelight targeting
     * @param hasTarget Whether Limelight has a valid target
     * @param targetBearing Target bearing from Limelight (degrees)
     * @param targetDistance Target distance from Limelight (inches)
     */
    public void autoAim(boolean hasTarget, double targetBearing, double targetDistance) {
        isAutoAiming = true;
        
        if (hasTarget) {
            // Update targeting data
            lastKnownTargetBearing = targetBearing;
            lastKnownTargetDistance = targetDistance;
            targetLostTimer.reset();
            
            // Set turret angle based on bearing
            setTurretAngle(targetBearing);
            
            // Calculate hood angle based on distance
            // TODO: Create lookup table or formula for hood angle vs distance
            double hoodAngle = calculateHoodAngleForDistance(targetDistance);
            setHoodAngle(hoodAngle);
            
        } else {
            // Target lost - use fallback odometry targeting if timeout not exceeded
            if (targetLostTimer.seconds() < TARGET_LOST_TIMEOUT) {
                // TODO: Implement odometry-based targeting fallback
                // Use last known target position and current robot position to calculate bearing
                // For now, hold last known angles
            } else {
                // Timeout exceeded, stop auto-aiming
                isAutoAiming = false;
            }
        }
    }
    
    /**
     * Calculate hood angle based on target distance
     * @param distance Distance to target in inches
     * @return Hood servo position (0.0 to 1.0)
     */
    private double calculateHoodAngleForDistance(double distance) {
        // TODO: Create a proper lookup table or polynomial fit
        // This is a simple linear approximation as a placeholder
        
        // Example: closer targets need lower hood angle, farther targets need higher hood angle
        double minDistance = 24.0;  // inches
        double maxDistance = 120.0; // inches
        
        // Clamp distance
        distance = Math.max(minDistance, Math.min(maxDistance, distance));
        
        // Linear interpolation
        double ratio = (distance - minDistance) / (maxDistance - minDistance);
        return HOOD_MIN_POSITION + ratio * (HOOD_MAX_POSITION - HOOD_MIN_POSITION);
    }
    
    /**
     * Disable auto-aim mode
     */
    public void disableAutoAim() {
        isAutoAiming = false;
    }
    
    /**
     * Check if shooter is at target velocity
     * @param tolerance Tolerance in ticks per second
     * @return True if shooter is at target velocity
     */
    public boolean isShooterAtVelocity(double tolerance) {
        if (targetShooterVelocity == 0) return false;
        
        double currentVelocity1 = hardware.shooterMotor1.getVelocity();
        double currentVelocity2 = hardware.shooterMotor2.getVelocity();
        double avgVelocity = (currentVelocity1 + currentVelocity2) / 2.0;
        
        return Math.abs(targetShooterVelocity - avgVelocity) < tolerance;
    }
    
    /**
     * Check if turret is at target angle
     * @param tolerance Tolerance in degrees
     * @return True if turret is at target angle
     */
    public boolean isTurretAtAngle(double tolerance) {
        double currentAngle = getTurretAngle();
        double error = Math.abs(targetTurretAngle - currentAngle);
        
        // Handle wrap-around
        if (error > 180) error = 360 - error;
        
        return error < tolerance;
    }
    
    /**
     * Get telemetry data for debugging
     */
    public String getTelemetry() {
        double currentVelocity1 = hardware.shooterMotor1.getVelocity();
        double currentVelocity2 = hardware.shooterMotor2.getVelocity();
        
        return String.format("Turret: %.1f° (target: %.1f°) | Shooter: %.0f/%.0f TPS | Hood: %.2f | Auto-aim: %s",
            getTurretAngle(),
            targetTurretAngle,
            (currentVelocity1 + currentVelocity2) / 2.0,
            targetShooterVelocity,
            targetHoodAngle,
            isAutoAiming ? "ON" : "OFF");
    }
}
