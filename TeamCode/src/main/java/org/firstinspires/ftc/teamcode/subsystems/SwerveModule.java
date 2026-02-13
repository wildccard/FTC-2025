package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

/**
 * One wheel module: drive motor + steering continuous servo + absolute analog encoder.
 */
public class SwerveModule {
    private static final double TWO_PI = 2.0 * Math.PI;

    private final DcMotorEx driveMotor;
    private final CRServo steerServo;
    private final AnalogInput steerEncoder;

    private double encoderOffsetRad;
    private double steeringKp = 1.8;

    public SwerveModule(DcMotorEx driveMotor, CRServo steerServo, AnalogInput steerEncoder, double encoderOffsetRad) {
        this.driveMotor = driveMotor;
        this.steerServo = steerServo;
        this.steerEncoder = steerEncoder;
        this.encoderOffsetRad = encoderOffsetRad;
    }

    public void setSteeringKp(double steeringKp) {
        this.steeringKp = steeringKp;
    }

    public void setEncoderOffsetRad(double encoderOffsetRad) {
        this.encoderOffsetRad = encoderOffsetRad;
    }

    public double getCurrentAngleRad() {
        double voltage = steerEncoder.getVoltage();
        double fraction = voltage / steerEncoder.getMaxVoltage();
        return wrapToPi((fraction * TWO_PI) - encoderOffsetRad);
    }

    public void setDesiredState(double wheelSpeed, double targetAngleRad) {
        double angleError = wrapToPi(targetAngleRad - getCurrentAngleRad());

        // Optimize to avoid rotating > 90° by reversing wheel direction.
        if (Math.abs(angleError) > Math.PI / 2.0) {
            angleError = wrapToPi(angleError + Math.PI);
            wheelSpeed = -wheelSpeed;
        }

        double steerPower = Range.clip(angleError * steeringKp, -1.0, 1.0);
        steerServo.setPower(steerPower);
        driveMotor.setPower(Range.clip(wheelSpeed, -1.0, 1.0));
    }

    public void stop() {
        steerServo.setPower(0.0);
        driveMotor.setPower(0.0);
    }

    private static double wrapToPi(double radians) {
        while (radians > Math.PI) radians -= TWO_PI;
        while (radians < -Math.PI) radians += TWO_PI;
        return radians;
    }
}
