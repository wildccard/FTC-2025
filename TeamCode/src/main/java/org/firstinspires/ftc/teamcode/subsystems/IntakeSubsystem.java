package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Dual goBILDA 5202 intake, one motor reversed in wiring/config.
 */
public class IntakeSubsystem {
    private final DcMotorEx leftIntake;
    private final DcMotorEx rightIntake;

    public IntakeSubsystem(DcMotorEx leftIntake, DcMotorEx rightIntake) {
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;

        this.leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void run(double power) {
        leftIntake.setPower(power);
        rightIntake.setPower(power);
    }

    public void stop() {
        run(0.0);
    }
}
