package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * SwerveDrive subsystem for field-centric swerve drive control
 * Uses 4 swerve modules with absolute encoders
 */
public class SwerveDrive {
    
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    
    // Robot dimensions
    // TODO: Measure and update these values for your robot
    private static final double TRACK_WIDTH = 0.4;  // meters, distance between left and right wheels
    private static final double WHEEL_BASE = 0.4;   // meters, distance between front and back wheels
    
    // IMU or gyro for field-centric control
    // In a real implementation, this would be initialized from hardware
    private double robotHeading = 0.0;  // Current robot heading in radians
    
    // Speed scaling factors
    // TODO: Tune these values
    private double maxSpeed = 1.0;
    private double maxRotationSpeed = 1.0;
    
    /**
     * Constructor for SwerveDrive
     * @param hardware RobotHardware instance with initialized hardware
     */
    public SwerveDrive(RobotHardware hardware) {
        // Initialize swerve modules with encoder offsets
        // TODO: Calibrate these encoder offsets for each module
        frontLeft = new SwerveModule(
            hardware.frontLeftDrive,
            hardware.frontLeftSteer,
            hardware.frontLeftEncoder,
            0.0  // Encoder offset in radians
        );
        
        frontRight = new SwerveModule(
            hardware.frontRightDrive,
            hardware.frontRightSteer,
            hardware.frontRightEncoder,
            0.0
        );
        
        backLeft = new SwerveModule(
            hardware.backLeftDrive,
            hardware.backLeftSteer,
            hardware.backLeftEncoder,
            0.0
        );
        
        backRight = new SwerveModule(
            hardware.backRightDrive,
            hardware.backRightSteer,
            hardware.backRightEncoder,
            0.0
        );
        
        // Set PID constants for all modules
        // TODO: Tune these PID values
        double kP = 0.01, kI = 0.0, kD = 0.0;
        frontLeft.setPIDConstants(kP, kI, kD);
        frontRight.setPIDConstants(kP, kI, kD);
        backLeft.setPIDConstants(kP, kI, kD);
        backRight.setPIDConstants(kP, kI, kD);
    }
    
    /**
     * Drive the robot using field-centric control
     * @param strafeX Strafe speed in X direction (-1.0 to 1.0)
     * @param strafeY Strafe speed in Y direction (-1.0 to 1.0)
     * @param rotation Rotation speed (-1.0 to 1.0)
     * @param fieldCentric Whether to use field-centric control
     */
    public void drive(double strafeX, double strafeY, double rotation, boolean fieldCentric) {
        // Apply field-centric transformation if enabled
        double x = strafeX;
        double y = strafeY;
        
        if (fieldCentric) {
            // Rotate the input by the robot's heading
            double temp = x * Math.cos(robotHeading) + y * Math.sin(robotHeading);
            y = -x * Math.sin(robotHeading) + y * Math.cos(robotHeading);
            x = temp;
        }
        
        // Scale inputs
        x *= maxSpeed;
        y *= maxSpeed;
        rotation *= maxRotationSpeed;
        
        // Calculate wheel vectors
        // Using swerve drive kinematics
        double r = Math.sqrt(TRACK_WIDTH * TRACK_WIDTH + WHEEL_BASE * WHEEL_BASE);
        
        // Front left
        double flX = x - rotation * (TRACK_WIDTH / r);
        double flY = y - rotation * (WHEEL_BASE / r);
        double flSpeed = Math.sqrt(flX * flX + flY * flY);
        double flAngle = Math.atan2(flY, flX);
        
        // Front right
        double frX = x + rotation * (TRACK_WIDTH / r);
        double frY = y - rotation * (WHEEL_BASE / r);
        double frSpeed = Math.sqrt(frX * frX + frY * frY);
        double frAngle = Math.atan2(frY, frX);
        
        // Back left
        double blX = x - rotation * (TRACK_WIDTH / r);
        double blY = y + rotation * (WHEEL_BASE / r);
        double blSpeed = Math.sqrt(blX * blX + blY * blY);
        double blAngle = Math.atan2(blY, blX);
        
        // Back right
        double brX = x + rotation * (TRACK_WIDTH / r);
        double brY = y + rotation * (WHEEL_BASE / r);
        double brSpeed = Math.sqrt(brX * brX + brY * brY);
        double brAngle = Math.atan2(brY, brX);
        
        // Normalize speeds if any exceed 1.0
        double maxWheelSpeed = Math.max(Math.max(flSpeed, frSpeed), Math.max(blSpeed, brSpeed));
        if (maxWheelSpeed > 1.0) {
            flSpeed /= maxWheelSpeed;
            frSpeed /= maxWheelSpeed;
            blSpeed /= maxWheelSpeed;
            brSpeed /= maxWheelSpeed;
        }
        
        // Set module states
        frontLeft.setTargetState(flAngle, flSpeed);
        frontRight.setTargetState(frAngle, frSpeed);
        backLeft.setTargetState(blAngle, blSpeed);
        backRight.setTargetState(brAngle, brSpeed);
    }
    
    /**
     * Drive using gamepad input
     * @param gamepad Gamepad to read from
     */
    public void driveWithGamepad(Gamepad gamepad) {
        // Left stick for translation, right stick X for rotation
        double strafeX = -gamepad.left_stick_x;
        double strafeY = -gamepad.left_stick_y;
        double rotation = -gamepad.right_stick_x;
        
        // Use field-centric control by default
        drive(strafeX, strafeY, rotation, true);
    }
    
    /**
     * Stop all modules
     */
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
    
    /**
     * Update robot heading (should be called with IMU/gyro reading)
     * @param heading Current robot heading in radians
     */
    public void updateHeading(double heading) {
        this.robotHeading = heading;
    }
    
    /**
     * Reset robot heading to zero
     */
    public void resetHeading() {
        this.robotHeading = 0.0;
    }
    
    /**
     * Set speed scaling factors
     * @param speed Max translation speed (0.0 to 1.0)
     * @param rotation Max rotation speed (0.0 to 1.0)
     */
    public void setSpeedScaling(double speed, double rotation) {
        this.maxSpeed = Math.max(0.0, Math.min(1.0, speed));
        this.maxRotationSpeed = Math.max(0.0, Math.min(1.0, rotation));
    }
    
    /**
     * Get module positions for debugging
     * @return Array of module absolute positions
     */
    public double[] getModulePositions() {
        return new double[] {
            frontLeft.getAbsolutePosition(),
            frontRight.getAbsolutePosition(),
            backLeft.getAbsolutePosition(),
            backRight.getAbsolutePosition()
        };
    }
}
