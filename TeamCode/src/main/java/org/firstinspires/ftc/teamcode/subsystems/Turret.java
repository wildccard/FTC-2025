package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Turret subsystem with auto-aim using Limelight (AprilTag)
 * Falls back to odometry targeting if target is lost
 * Adjustable hood angle based on distance
 */
public class Turret {
    
    private final DcMotorEx shooterMotor1;
    private final DcMotorEx shooterMotor2;
    private final Servo turretServo1;
    private final Servo turretServo2;
    private final Servo hoodServo;
    private final DcMotorEx turretEncoder;
    
    // PID Constants for turret aiming
    // TODO: Tune these values
    private double turretKP = 0.01;
    private double turretKI = 0.0;
    private double turretKD = 0.0;
    
    // Shooter speed constants
    // TODO: Tune these values based on testing
    private static final double MIN_SHOOTER_SPEED = 0.5;
    private static final double MAX_SHOOTER_SPEED = 1.0;
    
    // Hood angle mapping (distance to angle)
    // TODO: Calibrate based on testing
    private static final double MIN_HOOD_ANGLE = 0.2;  // Servo position for close shots
    private static final double MAX_HOOD_ANGLE = 0.8;  // Servo position for far shots
    private static final double MIN_DISTANCE = 0.5;    // meters
    private static final double MAX_DISTANCE = 5.0;    // meters
    
    // Target tracking
    private double targetBearing = 0.0;  // Target bearing from Limelight (degrees)
    private double targetDistance = 0.0; // Target distance (meters)
    private boolean targetLocked = false;
    
    // Odometry fallback position
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;
    
    // Target position (for odometry fallback)
    // TODO: Set based on field layout
    private static final double TARGET_X = 0.0;  // meters
    private static final double TARGET_Y = 0.0;  // meters
    
    // PID state
    private double integratedError = 0.0;
    private double previousError = 0.0;
    
    /**
     * Constructor for Turret subsystem
     * @param hardware RobotHardware instance
     */
    public Turret(RobotHardware hardware) {
        this.shooterMotor1 = hardware.shooterMotor1;
        this.shooterMotor2 = hardware.shooterMotor2;
        this.turretServo1 = hardware.turretServo1;
        this.turretServo2 = hardware.turretServo2;
        this.hoodServo = hardware.hoodServo;
        this.turretEncoder = hardware.turretEncoder;
        
        // Initialize hood to mid position
        hoodServo.setPosition(0.5);
    }
    
    /**
     * Update target information from Limelight
     * @param bearing Target bearing in degrees
     * @param distance Target distance in meters
     * @param hasTarget Whether a target is detected
     */
    public void updateLimelightTarget(double bearing, double distance, boolean hasTarget) {
        this.targetBearing = bearing;
        this.targetDistance = distance;
        this.targetLocked = hasTarget;
    }
    
    /**
     * Update robot position for odometry fallback
     * @param x Robot X position (meters)
     * @param y Robot Y position (meters)
     * @param heading Robot heading (radians)
     */
    public void updateRobotPosition(double x, double y, double heading) {
        this.robotX = x;
        this.robotY = y;
        this.robotHeading = heading;
    }
    
    /**
     * Calculate target bearing and distance using odometry
     */
    private void calculateOdometryTarget() {
        double dx = TARGET_X - robotX;
        double dy = TARGET_Y - robotY;
        
        targetDistance = Math.sqrt(dx * dx + dy * dy);
        
        // Calculate bearing relative to robot
        double absoluteBearing = Math.atan2(dy, dx);
        targetBearing = Math.toDegrees(absoluteBearing - robotHeading);
        
        // Normalize to -180 to 180
        while (targetBearing > 180) targetBearing -= 360;
        while (targetBearing < -180) targetBearing += 360;
    }
    
    /**
     * Aim turret at target using auto-aim
     */
    public void autoAim() {
        // If no Limelight target, use odometry
        if (!targetLocked) {
            calculateOdometryTarget();
        }
        
        // PID control for turret aiming
        double error = targetBearing;
        integratedError += error;
        double derivative = error - previousError;
        previousError = error;
        
        double turretPower = turretKP * error + turretKI * integratedError + turretKD * derivative;
        
        // Clamp turret power
        turretPower = Math.max(-0.5, Math.min(0.5, turretPower));
        
        // Set turret servos (synchronized movement)
        turretServo1.setPosition(0.5 + turretPower);
        turretServo2.setPosition(0.5 + turretPower);
        
        // Adjust hood based on distance
        adjustHood(targetDistance);
    }
    
    /**
     * Manually control turret position
     * @param power Turret rotation power (-1.0 to 1.0)
     */
    public void manualAim(double power) {
        // Reset PID
        integratedError = 0.0;
        previousError = 0.0;
        
        // Clamp power
        power = Math.max(-0.5, Math.min(0.5, power));
        
        turretServo1.setPosition(0.5 + power);
        turretServo2.setPosition(0.5 + power);
    }
    
    /**
     * Adjust hood angle based on target distance
     * @param distance Distance to target (meters)
     */
    private void adjustHood(double distance) {
        // Clamp distance to valid range
        distance = Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, distance));
        
        // Linear interpolation between min and max hood angles
        double ratio = (distance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE);
        double hoodPosition = MIN_HOOD_ANGLE + ratio * (MAX_HOOD_ANGLE - MIN_HOOD_ANGLE);
        
        hoodServo.setPosition(hoodPosition);
    }
    
    /**
     * Set hood angle manually
     * @param position Hood servo position (0.0 to 1.0)
     */
    public void setHoodAngle(double position) {
        position = Math.max(0.0, Math.min(1.0, position));
        hoodServo.setPosition(position);
    }
    
    /**
     * Set shooter speed
     * @param speed Shooter speed (0.0 to 1.0)
     */
    public void setShooterSpeed(double speed) {
        speed = Math.max(0.0, Math.min(1.0, speed));
        shooterMotor1.setPower(speed);
        shooterMotor2.setPower(speed);
    }
    
    /**
     * Spin up shooter to optimal speed based on distance
     */
    public void spinUpShooter() {
        // Calculate shooter speed based on distance
        // Farther targets need higher speeds
        double ratio = (targetDistance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE);
        ratio = Math.max(0.0, Math.min(1.0, ratio));
        
        double speed = MIN_SHOOTER_SPEED + ratio * (MAX_SHOOTER_SPEED - MIN_SHOOTER_SPEED);
        setShooterSpeed(speed);
    }
    
    /**
     * Stop shooter motors
     */
    public void stopShooter() {
        shooterMotor1.setPower(0.0);
        shooterMotor2.setPower(0.0);
    }
    
    /**
     * Check if turret is aimed at target (within tolerance)
     * @return True if turret is aimed
     */
    public boolean isAimed() {
        // TODO: Tune this tolerance
        return Math.abs(targetBearing) < 2.0;  // Within 2 degrees
    }
    
    /**
     * Get current turret encoder position
     * @return Turret encoder position
     */
    public int getTurretPosition() {
        return turretEncoder.getCurrentPosition();
    }
    
    /**
     * Check if target is locked
     * @return True if Limelight has a target
     */
    public boolean isTargetLocked() {
        return targetLocked;
    }
    
    /**
     * Get target distance
     * @return Distance to target in meters
     */
    public double getTargetDistance() {
        return targetDistance;
    }
    
    /**
     * Set PID constants for turret control
     * @param kP Proportional constant
     * @param kI Integral constant
     * @param kD Derivative constant
     */
    public void setPIDConstants(double kP, double kI, double kD) {
        this.turretKP = kP;
        this.turretKI = kI;
        this.turretKD = kD;
    }
}
