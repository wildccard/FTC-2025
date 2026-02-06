package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Sorter subsystem for intake and color-based ball sorting
 * Uses color sensors to detect ball color and lopata servos to sort
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
}
