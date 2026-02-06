package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.ArrayList;
import java.util.List;

/**
 * Sorter subsystem for intake and color-based ball sorting
 * Uses color sensors to detect ball color and lopata servos to sort
 * Can launch balls sequentially based on AprilTag decode instructions
 */
public class Sorter {
    
    private final DcMotorEx intakeMotor1;
    private final DcMotorEx intakeMotor2;
    
    private final ColorSensor colorSensor1;
    private final ColorSensor colorSensor2;
    private final ColorSensor colorSensor3;
    
    private final Servo lopataServo1;
    private final Servo lopataServo2;
    private final Servo lopataServo3;
    
    // Intake speed
    // TODO: Tune this value
    private static final double INTAKE_SPEED = 0.8;
    
    // Color detection thresholds
    // TODO: Calibrate these values based on your color sensors and game pieces
    private static final int RED_THRESHOLD = 200;
    private static final int BLUE_THRESHOLD = 200;
    private static final int RED_BLUE_RATIO = 2;  // Red/Blue ratio for red detection
    
    // Lopata servo positions
    // TODO: Calibrate these positions
    private static final double LOPATA_NEUTRAL = 0.5;
    private static final double LOPATA_SORT_LEFT = 0.2;
    private static final double LOPATA_SORT_RIGHT = 0.8;
    private static final double LOPATA_LAUNCH_UP = 1.0;  // Position to launch ball upward
    
    // Launch timing
    private static final long LAUNCH_DURATION_MS = 500;  // Time to hold launch position
    
    // Ball color enumeration
    public enum BallColor {
        NONE,
        RED,
        BLUE,
        UNKNOWN
    }
    
    // Target color to collect (set based on alliance)
    private BallColor targetColor = BallColor.RED;
    
    // Intake state
    private boolean intakeRunning = false;
    private boolean autoSortEnabled = true;
    
    // Sequential launching state
    private List<BallColor> launchSequence = new ArrayList<>();  // AprilTag decoded sequence
    private List<BallColor> sortedBalls = new ArrayList<>();     // Balls sorted and ready to launch
    private int launchIndex = 0;                                  // Current position in launch sequence
    private boolean isLaunching = false;                          // Whether we're in launch mode
    private long launchStartTime = 0;                             // Time when current launch started
    private int currentLaunchServo = -1;                          // Which servo is currently launching (-1 = none)
    
    /**
     * Constructor for Sorter subsystem
     * @param hardware RobotHardware instance
     */
    public Sorter(RobotHardware hardware) {
        this.intakeMotor1 = hardware.intakeMotor1;
        this.intakeMotor2 = hardware.intakeMotor2;
        
        this.colorSensor1 = hardware.colorSensor1;
        this.colorSensor2 = hardware.colorSensor2;
        this.colorSensor3 = hardware.colorSensor3;
        
        this.lopataServo1 = hardware.lopataServo1;
        this.lopataServo2 = hardware.lopataServo2;
        this.lopataServo3 = hardware.lopataServo3;
        
        // Initialize lopata servos to neutral position
        resetLopataServos();
    }
    
    /**
     * Start intake motors
     */
    public void startIntake() {
        intakeMotor1.setPower(INTAKE_SPEED);
        intakeMotor2.setPower(INTAKE_SPEED);
        intakeRunning = true;
    }
    
    /**
     * Stop intake motors
     */
    public void stopIntake() {
        intakeMotor1.setPower(0.0);
        intakeMotor2.setPower(0.0);
        intakeRunning = false;
    }
    
    /**
     * Reverse intake motors (eject)
     */
    public void reverseIntake() {
        intakeMotor1.setPower(-INTAKE_SPEED);
        intakeMotor2.setPower(-INTAKE_SPEED);
        intakeRunning = true;
    }
    
    /**
     * Set intake speed manually
     * @param speed Intake speed (-1.0 to 1.0)
     */
    public void setIntakeSpeed(double speed) {
        speed = Math.max(-1.0, Math.min(1.0, speed));
        intakeMotor1.setPower(speed);
        intakeMotor2.setPower(speed);
        intakeRunning = (speed != 0.0);
    }
    
    /**
     * Detect ball color from a color sensor
     * @param sensor ColorSensor to read from
     * @return Detected ball color
     */
    private BallColor detectBallColor(ColorSensor sensor) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();
        
        // Check if any color is detected above threshold
        if (red < RED_THRESHOLD && blue < BLUE_THRESHOLD) {
            return BallColor.NONE;
        }
        
        // Determine if it's red or blue based on ratio
        if (red > blue * RED_BLUE_RATIO) {
            return BallColor.RED;
        } else if (blue > red * RED_BLUE_RATIO) {
            return BallColor.BLUE;
        }
        
        return BallColor.UNKNOWN;
    }
    
    /**
     * Update sorting logic based on color sensor readings
     * This should be called periodically in the main loop
     */
    public void updateSorting() {
        if (!autoSortEnabled) {
            return;
        }
        
        // Check each color sensor and actuate corresponding lopata servo
        BallColor color1 = detectBallColor(colorSensor1);
        BallColor color2 = detectBallColor(colorSensor2);
        BallColor color3 = detectBallColor(colorSensor3);
        
        // Actuate lopata servos based on detected colors
        // Sort to right if target color, left if not
        actuateLopata(lopataServo1, color1);
        actuateLopata(lopataServo2, color2);
        actuateLopata(lopataServo3, color3);
    }
    
    /**
     * Actuate a single lopata servo based on ball color
     * @param servo Lopata servo to actuate
     * @param color Detected ball color
     */
    private void actuateLopata(Servo servo, BallColor color) {
        if (color == BallColor.NONE) {
            // No ball detected, return to neutral
            servo.setPosition(LOPATA_NEUTRAL);
        } else if (color == targetColor) {
            // Target color, sort to right (collect)
            servo.setPosition(LOPATA_SORT_RIGHT);
        } else if (color != BallColor.UNKNOWN) {
            // Wrong color, sort to left (reject)
            servo.setPosition(LOPATA_SORT_LEFT);
        } else {
            // Unknown color, default to neutral or collect
            servo.setPosition(LOPATA_NEUTRAL);
        }
    }
    
    /**
     * Reset all lopata servos to neutral position
     */
    public void resetLopataServos() {
        lopataServo1.setPosition(LOPATA_NEUTRAL);
        lopataServo2.setPosition(LOPATA_NEUTRAL);
        lopataServo3.setPosition(LOPATA_NEUTRAL);
    }
    
    /**
     * Manually set a lopata servo position
     * @param servoIndex Servo index (1-3)
     * @param position Servo position (0.0 to 1.0)
     */
    public void setLopataPosition(int servoIndex, double position) {
        position = Math.max(0.0, Math.min(1.0, position));
        
        switch (servoIndex) {
            case 1:
                lopataServo1.setPosition(position);
                break;
            case 2:
                lopataServo2.setPosition(position);
                break;
            case 3:
                lopataServo3.setPosition(position);
                break;
        }
    }
    
    /**
     * Set target color to collect
     * @param color Target ball color
     */
    public void setTargetColor(BallColor color) {
        this.targetColor = color;
    }
    
    /**
     * Enable or disable automatic sorting
     * @param enabled True to enable auto-sorting
     */
    public void setAutoSortEnabled(boolean enabled) {
        this.autoSortEnabled = enabled;
        if (!enabled) {
            resetLopataServos();
        }
    }
    
    /**
     * Check if intake is running
     * @return True if intake is running
     */
    public boolean isIntakeRunning() {
        return intakeRunning;
    }
    
    /**
     * Get color sensor readings for debugging
     * @return Array of color sensor readings [sensor1_red, sensor1_blue, sensor2_red, sensor2_blue, sensor3_red, sensor3_blue]
     */
    public int[] getColorSensorReadings() {
        return new int[] {
            colorSensor1.red(), colorSensor1.blue(),
            colorSensor2.red(), colorSensor2.blue(),
            colorSensor3.red(), colorSensor3.blue()
        };
    }
    
    /**
     * Get detected colors from all sensors
     * @return Array of detected colors
     */
    public BallColor[] getDetectedColors() {
        return new BallColor[] {
            detectBallColor(colorSensor1),
            detectBallColor(colorSensor2),
            detectBallColor(colorSensor3)
        };
    }
    
    // ========== SEQUENTIAL LAUNCHING METHODS ==========
    
    /**
     * Set the launch sequence from AprilTag decode
     * This should be called at initialization with the decoded sequence
     * @param sequence List of ball colors in the order they should be launched
     */
    public void setLaunchSequence(List<BallColor> sequence) {
        this.launchSequence = new ArrayList<>(sequence);
        this.launchIndex = 0;
        telemetryMessage("Launch sequence set: " + sequence.size() + " balls");
    }
    
    /**
     * Set launch sequence from AprilTag ID
     * Decodes the AprilTag ID into a ball color sequence
     * @param aprilTagId The AprilTag ID detected (0-15 typical range)
     */
    public void setLaunchSequenceFromAprilTag(int aprilTagId) {
        launchSequence.clear();
        
        // Decode AprilTag ID into sequence
        // Example decoding logic (customize based on FTC game rules):
        // Each bit represents a ball: 0=RED, 1=BLUE
        // Or use lookup table for specific tag IDs
        
        switch (aprilTagId) {
            case 1:
                // Example: RED, RED, BLUE
                launchSequence.add(BallColor.RED);
                launchSequence.add(BallColor.RED);
                launchSequence.add(BallColor.BLUE);
                break;
            case 2:
                // Example: BLUE, RED, BLUE
                launchSequence.add(BallColor.BLUE);
                launchSequence.add(BallColor.RED);
                launchSequence.add(BallColor.BLUE);
                break;
            case 3:
                // Example: RED, BLUE, RED
                launchSequence.add(BallColor.RED);
                launchSequence.add(BallColor.BLUE);
                launchSequence.add(BallColor.RED);
                break;
            default:
                // Default sequence: alternating RED, BLUE, RED
                launchSequence.add(BallColor.RED);
                launchSequence.add(BallColor.BLUE);
                launchSequence.add(BallColor.RED);
                break;
        }
        
        this.launchIndex = 0;
        telemetryMessage("Launch sequence from AprilTag " + aprilTagId + ": " + launchSequence.size() + " balls");
    }
    
    /**
     * Track a sorted ball (called when a ball is successfully sorted)
     * @param color Color of the sorted ball
     */
    public void trackSortedBall(BallColor color) {
        if (color != BallColor.NONE && color != BallColor.UNKNOWN) {
            sortedBalls.add(color);
            telemetryMessage("Ball sorted: " + color + " (Total: " + sortedBalls.size() + ")");
        }
    }
    
    /**
     * Start sequential launching of balls based on the AprilTag sequence
     * Balls will be launched in the order specified by the launch sequence
     */
    public void startSequentialLaunch() {
        if (launchSequence.isEmpty()) {
            telemetryMessage("Error: No launch sequence set!");
            return;
        }
        
        isLaunching = true;
        launchIndex = 0;
        telemetryMessage("Starting sequential launch...");
    }
    
    /**
     * Stop sequential launching
     */
    public void stopSequentialLaunch() {
        isLaunching = false;
        currentLaunchServo = -1;
        resetLopataServos();
        telemetryMessage("Sequential launch stopped");
    }
    
    /**
     * Update sequential launch logic
     * This should be called periodically in the main loop when launching
     */
    public void updateSequentialLaunch() {
        if (!isLaunching) {
            return;
        }
        
        // Check if we're currently launching a ball
        if (currentLaunchServo != -1) {
            // Check if launch duration has elapsed
            if (System.currentTimeMillis() - launchStartTime >= LAUNCH_DURATION_MS) {
                // Return servo to neutral
                setLopataPosition(currentLaunchServo, LOPATA_NEUTRAL);
                currentLaunchServo = -1;
                launchIndex++;
                
                // Small delay before next launch
                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
            return;
        }
        
        // Check if we've completed the sequence
        if (launchIndex >= launchSequence.size()) {
            telemetryMessage("Launch sequence complete!");
            stopSequentialLaunch();
            return;
        }
        
        // Launch next ball in sequence
        BallColor nextColor = launchSequence.get(launchIndex);
        int servoToLaunch = findBallToLaunch(nextColor);
        
        if (servoToLaunch != -1) {
            launchBall(servoToLaunch);
            telemetryMessage("Launching ball " + (launchIndex + 1) + "/" + launchSequence.size() + ": " + nextColor);
        } else {
            telemetryMessage("Warning: No " + nextColor + " ball available to launch!");
            launchIndex++;  // Skip to next in sequence
        }
    }
    
    /**
     * Find which servo has the next ball to launch
     * @param color Color of ball to find
     * @return Servo index (1-3) or -1 if not found
     */
    private int findBallToLaunch(BallColor color) {
        // Check each color sensor for the requested color
        BallColor[] detectedColors = getDetectedColors();
        
        for (int i = 0; i < detectedColors.length; i++) {
            if (detectedColors[i] == color) {
                return i + 1;  // Return servo index (1-based)
            }
        }
        
        // If not currently detected, check sorted balls list
        // Assume balls are distributed across servos
        if (!sortedBalls.isEmpty() && sortedBalls.contains(color)) {
            // Return first available servo (in practice, track ball positions)
            return 1;
        }
        
        return -1;  // No ball of this color found
    }
    
    /**
     * Launch a ball from a specific servo
     * @param servoIndex Servo index (1-3)
     */
    private void launchBall(int servoIndex) {
        currentLaunchServo = servoIndex;
        launchStartTime = System.currentTimeMillis();
        setLopataPosition(servoIndex, LOPATA_LAUNCH_UP);
    }
    
    /**
     * Get launch sequence status
     * @return String describing current launch status
     */
    public String getLaunchStatus() {
        if (!isLaunching) {
            return "Ready (" + launchSequence.size() + " balls queued)";
        }
        return "Launching " + (launchIndex + 1) + "/" + launchSequence.size();
    }
    
    /**
     * Check if currently in launch mode
     * @return True if launching
     */
    public boolean isLaunching() {
        return isLaunching;
    }
    
    /**
     * Get the launch sequence
     * @return List of ball colors in launch order
     */
    public List<BallColor> getLaunchSequence() {
        return new ArrayList<>(launchSequence);
    }
    
    /**
     * Get number of sorted balls
     * @return Count of sorted balls
     */
    public int getSortedBallCount() {
        return sortedBalls.size();
    }
    
    /**
     * Helper method for telemetry messages (to be overridden or logged)
     * @param message Message to log
     */
    private void telemetryMessage(String message) {
        // This would be connected to the OpMode telemetry in practice
        // For now, just print to system
        System.out.println("[Sorter] " + message);
    }
}
