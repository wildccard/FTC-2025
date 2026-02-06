package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;

/**
 * TeleOpMain - Main TeleOp OpMode for the 2025 FTC Robot
 * 
 * Controls:
 * Gamepad 1 (Driver):
 *   - Left Stick: Strafe (X/Y translation)
 *   - Right Stick X: Rotation
 *   - A: Reset field-centric heading
 *   - B: Toggle speed mode (slow/normal)
 * 
 * Gamepad 2 (Operator):
 *   - Right Bumper: Start intake
 *   - Left Bumper: Reverse intake
 *   - Release Bumpers: Stop intake
 *   - A: Auto-aim turret and spin up shooter
 *   - B: Stop shooter
 *   - X: Manual turret left
 *   - Y: Manual turret right
 *   - D-pad Up: Increase hood angle
 *   - D-pad Down: Decrease hood angle
 *   - Right Trigger: Set shooter speed
 */
@TeleOp(name="TeleOpMain", group="Competition")
public class TeleOpMain extends OpMode {
    
    // Hardware and subsystems
    private RobotHardware robot;
    private SwerveDrive swerveDrive;
    private Turret turret;
    private Sorter sorter;
    
    // Timers
    private ElapsedTime runtime = new ElapsedTime();
    
    // State variables
    private boolean slowMode = false;
    private boolean autoAimEnabled = false;
    private boolean lastAButton = false;
    private boolean lastBButton = false;
    
    // Hood angle control
    private double hoodAngle = 0.5;
    private static final double HOOD_INCREMENT = 0.02;
    
    /**
     * Initialize hardware and subsystems
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        
        // Initialize hardware
        robot = new RobotHardware();
        robot.init(hardwareMap);
        
        // Initialize subsystems
        swerveDrive = new SwerveDrive(robot);
        turret = new Turret(robot);
        sorter = new Sorter(robot);
        
        // Set initial target color based on alliance
        // TODO: Determine alliance color programmatically or via configuration
        sorter.setTargetColor(Sorter.BallColor.RED);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Mode", "Waiting for start");
        telemetry.update();
    }
    
    /**
     * Initialize on start
     */
    @Override
    public void init_loop() {
        telemetry.addData("Status", "Ready to start");
        telemetry.addData("Alliance", "Configure target color in code");
        telemetry.update();
    }
    
    /**
     * Start of TeleOp
     */
    @Override
    public void start() {
        runtime.reset();
        swerveDrive.resetHeading();
    }
    
    /**
     * Main TeleOp loop
     */
    @Override
    public void loop() {
        // ========== GAMEPAD 1: DRIVER CONTROLS ==========
        
        // Swerve drive control
        swerveDrive.driveWithGamepad(gamepad1);
        
        // Reset field-centric heading
        if (gamepad1.a) {
            swerveDrive.resetHeading();
        }
        
        // Toggle speed mode
        if (gamepad1.b && !lastBButton) {
            slowMode = !slowMode;
            if (slowMode) {
                swerveDrive.setSpeedScaling(0.4, 0.4);
            } else {
                swerveDrive.setSpeedScaling(1.0, 1.0);
            }
        }
        lastBButton = gamepad1.b;
        
        // ========== GAMEPAD 2: OPERATOR CONTROLS ==========
        
        // Intake control
        if (gamepad2.right_bumper) {
            sorter.startIntake();
        } else if (gamepad2.left_bumper) {
            sorter.reverseIntake();
        } else if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
            // Only stop if neither bumper is pressed
            if (sorter.isIntakeRunning()) {
                sorter.stopIntake();
            }
        }
        
        // Auto-aim and shooter control
        if (gamepad2.a && !lastAButton) {
            autoAimEnabled = !autoAimEnabled;
        }
        lastAButton = gamepad2.a;
        
        if (autoAimEnabled) {
            // Auto-aim turret
            turret.autoAim();
            
            // Spin up shooter when aimed
            if (turret.isAimed()) {
                turret.spinUpShooter();
            }
        } else {
            // Manual turret control
            double turretPower = 0.0;
            if (gamepad2.x) {
                turretPower = -0.3;  // Left
            } else if (gamepad2.y) {
                turretPower = 0.3;   // Right
            }
            
            if (turretPower != 0.0) {
                turret.manualAim(turretPower);
            }
        }
        
        // Stop shooter
        if (gamepad2.b) {
            turret.stopShooter();
            autoAimEnabled = false;
        }
        
        // Manual shooter speed control with trigger
        if (gamepad2.right_trigger > 0.1) {
            turret.setShooterSpeed(gamepad2.right_trigger);
        }
        
        // Hood angle control
        if (gamepad2.dpad_up) {
            hoodAngle += HOOD_INCREMENT;
            hoodAngle = Math.min(1.0, hoodAngle);
            turret.setHoodAngle(hoodAngle);
        } else if (gamepad2.dpad_down) {
            hoodAngle -= HOOD_INCREMENT;
            hoodAngle = Math.max(0.0, hoodAngle);
            turret.setHoodAngle(hoodAngle);
        }
        
        // ========== SUBSYSTEM UPDATES ==========
        
        // Update sorting logic (color detection and lopata actuation)
        sorter.updateSorting();
        
        // Update turret with odometry data
        // TODO: Integrate actual odometry readings
        turret.updateRobotPosition(0.0, 0.0, 0.0);
        
        // Update Limelight data
        // TODO: Integrate actual Limelight readings
        // Example: turret.updateLimelightTarget(bearing, distance, hasTarget);
        
        // ========== TELEMETRY ==========
        updateTelemetry();
    }
    
    /**
     * Update telemetry data
     */
    private void updateTelemetry() {
        telemetry.addData("Runtime", "%.1f sec", runtime.seconds());
        telemetry.addData("Speed Mode", slowMode ? "SLOW" : "NORMAL");
        
        // Drive telemetry
        telemetry.addData("", "--- DRIVE ---");
        double[] modulePositions = swerveDrive.getModulePositions();
        telemetry.addData("FL Module", "%.2f rad", modulePositions[0]);
        telemetry.addData("FR Module", "%.2f rad", modulePositions[1]);
        telemetry.addData("BL Module", "%.2f rad", modulePositions[2]);
        telemetry.addData("BR Module", "%.2f rad", modulePositions[3]);
        
        // Turret telemetry
        telemetry.addData("", "--- TURRET ---");
        telemetry.addData("Auto-Aim", autoAimEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Target Locked", turret.isTargetLocked() ? "YES" : "NO");
        telemetry.addData("Target Distance", "%.2f m", turret.getTargetDistance());
        telemetry.addData("Aimed", turret.isAimed() ? "YES" : "NO");
        telemetry.addData("Hood Angle", "%.2f", hoodAngle);
        
        // Sorter telemetry
        telemetry.addData("", "--- SORTER ---");
        telemetry.addData("Intake", sorter.isIntakeRunning() ? "RUNNING" : "STOPPED");
        
        Sorter.BallColor[] colors = sorter.getDetectedColors();
        telemetry.addData("Sensor 1", colors[0].toString());
        telemetry.addData("Sensor 2", colors[1].toString());
        telemetry.addData("Sensor 3", colors[2].toString());
        
        telemetry.update();
    }
    
    /**
     * Stop all subsystems
     */
    @Override
    public void stop() {
        swerveDrive.stop();
        turret.stopShooter();
        sorter.stopIntake();
        sorter.resetLopataServos();
    }
}
