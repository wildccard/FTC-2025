package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;

/**
 * TeleOpMain - Main TeleOp program for the 2025 FTC robot
 * 
 * Controller 1 (Driver):
 * - Left Stick: Forward/Strafe movement
 * - Right Stick X: Rotation
 * - A Button: Toggle field-centric mode
 * - B Button: Reset field-centric heading
 * 
 * Controller 2 (Operator):
 * - Left Stick Y: Manual turret control
 * - Right Stick Y: Manual hood control
 * - Right Trigger: Run intake
 * - Left Trigger: Run outtake
 * - A Button: Toggle auto-aim mode
 * - B Button: Toggle shooter on/off
 * - X Button: Manually deploy lopata servo 1
 * - Y Button: Manually retract all lopatas
 * - DPad Up/Down: Adjust shooter velocity
 */
@TeleOp(name="TeleOp Main", group="Competition")
public class TeleOpMain extends OpMode {
    
    // Hardware and subsystems
    private RobotHardware robot = new RobotHardware();
    private SwerveDrive swerveDrive;
    private Turret turret;
    private Sorter sorter;
    
    // OpMode state
    private ElapsedTime runtime = new ElapsedTime();
    private boolean fieldCentricMode = true;
    private boolean autoAimMode = false;
    private boolean shooterEnabled = false;
    
    // Button state tracking for toggle functionality
    private boolean lastFieldCentricButton = false;
    private boolean lastResetHeadingButton = false;
    private boolean lastAutoAimButton = false;
    private boolean lastShooterButton = false;
    
    // Shooter velocity control
    // TODO: Tune this value for your robot
    private double shooterVelocity = 2000; // ticks per second
    private static final double SHOOTER_VELOCITY_INCREMENT = 100;
    
    // Limelight data (placeholder - replace with actual Limelight integration)
    private boolean limelightHasTarget = false;
    private double limelightTargetBearing = 0;
    private double limelightTargetDistance = 60;
    
    /**
     * Initialize the robot
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        
        // Initialize hardware
        robot.init(hardwareMap);
        
        // Initialize subsystems
        swerveDrive = new SwerveDrive(robot);
        turret = new Turret(robot);
        sorter = new Sorter(robot);
        
        // Set alliance color for sorter
        // TODO: Change this based on alliance or use a configuration
        sorter.setAllianceColor(false); // false = blue alliance
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Field-Centric", fieldCentricMode ? "ENABLED" : "DISABLED");
        telemetry.update();
    }
    
    /**
     * Initialize and reset field-centric heading
     */
    @Override
    public void init_loop() {
        telemetry.addData("Status", "Ready to start");
        telemetry.addData("Field-Centric", fieldCentricMode ? "ENABLED" : "DISABLED");
        telemetry.addData("Instructions", "Press START to begin");
        telemetry.update();
    }
    
    /**
     * Start the OpMode
     */
    @Override
    public void start() {
        runtime.reset();
        robot.resetIMU();
        swerveDrive.resetFieldCentric();
    }
    
    /**
     * Main loop - runs repeatedly during TeleOp
     */
    @Override
    public void loop() {
        // ========== DRIVER CONTROLS (Gamepad 1) ==========
        handleDriverControls();
        
        // ========== OPERATOR CONTROLS (Gamepad 2) ==========
        handleOperatorControls();
        
        // ========== UPDATE SUBSYSTEMS ==========
        turret.update();
        sorter.update();
        
        // ========== TELEMETRY ==========
        updateTelemetry();
    }
    
    /**
     * Handle driver controls (gamepad1)
     */
    private void handleDriverControls() {
        // Get drive inputs
        double forward = -gamepad1.left_stick_y;  // Inverted
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        
        // Apply deadzone
        forward = applyDeadzone(forward, 0.05);
        strafe = applyDeadzone(strafe, 0.05);
        rotate = applyDeadzone(rotate, 0.05);
        
        // Apply speed scaling (optional)
        double speedMultiplier = 1.0;
        if (gamepad1.left_bumper) {
            speedMultiplier = 0.5; // Slow mode
        } else if (gamepad1.right_bumper) {
            speedMultiplier = 1.0; // Fast mode
        }
        
        forward *= speedMultiplier;
        strafe *= speedMultiplier;
        rotate *= speedMultiplier;
        
        // Drive the robot
        swerveDrive.drive(forward, strafe, rotate, fieldCentricMode);
        
        // Toggle field-centric mode
        if (gamepad1.a && !lastFieldCentricButton) {
            fieldCentricMode = !fieldCentricMode;
        }
        lastFieldCentricButton = gamepad1.a;
        
        // Reset field-centric heading
        if (gamepad1.b && !lastResetHeadingButton) {
            robot.resetIMU();
            swerveDrive.resetFieldCentric();
        }
        lastResetHeadingButton = gamepad1.b;
    }
    
    /**
     * Handle operator controls (gamepad2)
     */
    private void handleOperatorControls() {
        // ========== INTAKE CONTROLS ==========
        if (gamepad2.right_trigger > 0.1) {
            // Run intake
            sorter.intake();
        } else if (gamepad2.left_trigger > 0.1) {
            // Run outtake
            sorter.outtake();
        } else {
            // Stop intake
            sorter.stopIntake();
        }
        
        // ========== TURRET CONTROLS ==========
        // Toggle auto-aim mode
        if (gamepad2.a && !lastAutoAimButton) {
            autoAimMode = !autoAimMode;
        }
        lastAutoAimButton = gamepad2.a;
        
        if (autoAimMode) {
            // Auto-aim using Limelight
            // TODO: Replace with actual Limelight data
            limelightHasTarget = false; // Placeholder
            turret.autoAim(limelightHasTarget, limelightTargetBearing, limelightTargetDistance);
        } else {
            // Manual turret control
            double turretInput = -gamepad2.left_stick_y;
            turretInput = applyDeadzone(turretInput, 0.05);
            
            if (Math.abs(turretInput) > 0.01) {
                double currentAngle = turret.getTurretAngle();
                double newAngle = currentAngle + turretInput * 2.0; // 2 degrees per loop
                turret.setTurretAngle(newAngle);
            }
            
            // Manual hood control
            double hoodInput = -gamepad2.right_stick_y;
            hoodInput = applyDeadzone(hoodInput, 0.05);
            
            if (Math.abs(hoodInput) > 0.01) {
                double currentHood = turret.getHoodAngle();
                double newHood = currentHood + hoodInput * 0.01; // Increment by 0.01
                turret.setHoodAngle(newHood);
            }
        }
        
        // ========== SHOOTER CONTROLS ==========
        // Toggle shooter on/off
        if (gamepad2.b && !lastShooterButton) {
            shooterEnabled = !shooterEnabled;
            if (shooterEnabled) {
                turret.setShooterVelocity(shooterVelocity);
            } else {
                turret.stopShooter();
            }
        }
        lastShooterButton = gamepad2.b;
        
        // Adjust shooter velocity
        if (gamepad2.dpad_up) {
            shooterVelocity += SHOOTER_VELOCITY_INCREMENT;
            if (shooterEnabled) {
                turret.setShooterVelocity(shooterVelocity);
            }
        } else if (gamepad2.dpad_down) {
            shooterVelocity = Math.max(0, shooterVelocity - SHOOTER_VELOCITY_INCREMENT);
            if (shooterEnabled) {
                turret.setShooterVelocity(shooterVelocity);
            }
        }
        
        // ========== LOPATA (SORTER) MANUAL CONTROLS ==========
        if (gamepad2.x) {
            // Manually deploy lopata 1
            sorter.deployLopata(1);
        }
        
        if (gamepad2.y) {
            // Retract all lopatas
            sorter.retractAllLopatas();
        }
    }
    
    /**
     * Apply deadzone to controller input
     */
    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0;
        }
        return value;
    }
    
    /**
     * Update telemetry
     */
    private void updateTelemetry() {
        telemetry.addData("Runtime", "%.2f", runtime.seconds());
        telemetry.addData("", ""); // Blank line
        
        // Drive telemetry
        telemetry.addData("Drive Mode", fieldCentricMode ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addData("Heading", "%.1f°", robot.getHeading());
        telemetry.addData("Swerve", swerveDrive.getTelemetry());
        telemetry.addData("", ""); // Blank line
        
        // Turret telemetry
        telemetry.addData("Auto-Aim", autoAimMode ? "ENABLED" : "DISABLED");
        telemetry.addData("Shooter", shooterEnabled ? "ON" : "OFF");
        telemetry.addData("Turret", turret.getTelemetry());
        telemetry.addData("", ""); // Blank line
        
        // Sorter telemetry
        telemetry.addData("Sorter", sorter.getTelemetry());
        telemetry.addData("Color Sensors", sorter.getColorSensorTelemetry());
        
        telemetry.update();
    }
    
    /**
     * Stop the robot when OpMode ends
     */
    @Override
    public void stop() {
        swerveDrive.stop();
        turret.stopShooter();
        sorter.stopIntake();
        sorter.retractAllLopatas();
    }
}
