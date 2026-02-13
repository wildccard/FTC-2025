package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.HeadingProvider;

/**
 * Turret with two continuous servos and a REV through-bore encoder on an encoder-capable port.
 */
public class TurretSubsystem {
    private final CRServo leftServo;
    private final CRServo rightServo;
    private final DcMotorEx encoderMotor;
    private final HeadingProvider headingProvider;

    private final ElapsedTime dt = new ElapsedTime();
    private int lastTicks;
    private double turretAngleRad;
    private double lastRobotHeadingRad;

    private static final double TICKS_PER_REV = 8192.0;
    private static final double GEAR_RATIO = 1.0; // encoder rev per turret rev

    private double kP = 2.1;
    private double kD = 0.08;
    private double previousError;

    public TurretSubsystem(CRServo leftServo, CRServo rightServo, DcMotorEx encoderMotor, HeadingProvider headingProvider) {
        this.leftServo = leftServo;
        this.rightServo = rightServo;
        this.encoderMotor = encoderMotor;
        this.headingProvider = headingProvider;
        this.lastTicks = encoderMotor.getCurrentPosition();
        this.lastRobotHeadingRad = headingProvider.getHeadingRadians();
    }

    public void update() {
        int ticks = encoderMotor.getCurrentPosition();
        int delta = ticks - lastTicks;
        lastTicks = ticks;

        turretAngleRad += (delta / TICKS_PER_REV) * (2.0 * Math.PI) / GEAR_RATIO;

        // compensate for robot rotation to keep world-frame aim stable
        double robotHeading = headingProvider.getHeadingRadians();
        turretAngleRad -= wrapToPi(robotHeading - lastRobotHeadingRad);
        lastRobotHeadingRad = robotHeading;
    }

    public double getTurretAngleRad() {
        return turretAngleRad;
    }

    public void aimAtTagYaw(double targetYawDeg) {
        double targetRad = Math.toRadians(targetYawDeg);
        double error = wrapToPi(targetRad - turretAngleRad);
        double deltaT = Math.max(0.005, dt.seconds());
        dt.reset();

        double derivative = (error - previousError) / deltaT;
        previousError = error;

        double out = Range.clip((kP * error) + (kD * derivative), -1.0, 1.0);
        leftServo.setPower(out);
        rightServo.setPower(out);
    }

    public void stop() {
        leftServo.setPower(0.0);
        rightServo.setPower(0.0);
    }

    private static double wrapToPi(double radians) {
        while (radians > Math.PI) radians -= 2.0 * Math.PI;
        while (radians < -Math.PI) radians += 2.0 * Math.PI;
        return radians;
    }
}
