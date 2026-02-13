package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Flywheel + hood angle subsystem.
 */
public class ShooterSubsystem {
    private final DcMotorEx flywheelLeft;
    private final DcMotorEx flywheelRight;
    private final Servo hoodServo;

    public ShooterSubsystem(DcMotorEx flywheelLeft, DcMotorEx flywheelRight, Servo hoodServo) {
        this.flywheelLeft = flywheelLeft;
        this.flywheelRight = flywheelRight;
        this.hoodServo = hoodServo;
    }

    public void setFlywheelPower(double power) {
        flywheelLeft.setPower(power);
        flywheelRight.setPower(power);
    }

    public void setHoodPosition(double pos) {
        hoodServo.setPosition(pos);
    }

    public void stop() {
        setFlywheelPower(0.0);
    }
}
