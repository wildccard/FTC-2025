package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.HeadingProvider;

/**
 * Field-centric 4 wheel swerve subsystem.
 */
public class SwerveDriveSubsystem {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final HeadingProvider headingProvider;

    // Robot geometry in arbitrary units (ratio matters, not absolute values).
    private final double wheelBase;
    private final double trackWidth;

    public SwerveDriveSubsystem(
            SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule backLeft,
            SwerveModule backRight,
            HeadingProvider headingProvider,
            double wheelBase,
            double trackWidth) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.headingProvider = headingProvider;
        this.wheelBase = wheelBase;
        this.trackWidth = trackWidth;
    }

    /**
     * @param xSpeed forward input [-1..1]
     * @param ySpeed left input [-1..1]
     * @param omega  CCW rotate input [-1..1]
     * @param fieldCentric true = joystick is field relative
     */
    public void drive(double xSpeed, double ySpeed, double omega, boolean fieldCentric) {
        if (fieldCentric) {
            double heading = headingProvider.getHeadingRadians();
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);
            double robotX = xSpeed * cos + ySpeed * sin;
            double robotY = -xSpeed * sin + ySpeed * cos;
            xSpeed = robotX;
            ySpeed = robotY;
        }

        double r = Math.hypot(wheelBase, trackWidth);
        double a = xSpeed - omega * (wheelBase / r);
        double b = xSpeed + omega * (wheelBase / r);
        double c = ySpeed - omega * (trackWidth / r);
        double d = ySpeed + omega * (trackWidth / r);

        double flSpeed = Math.hypot(b, d);
        double frSpeed = Math.hypot(b, c);
        double blSpeed = Math.hypot(a, d);
        double brSpeed = Math.hypot(a, c);

        double max = Math.max(1.0, Math.max(Math.max(flSpeed, frSpeed), Math.max(blSpeed, brSpeed)));
        flSpeed /= max;
        frSpeed /= max;
        blSpeed /= max;
        brSpeed /= max;

        frontLeft.setDesiredState(flSpeed, Math.atan2(d, b));
        frontRight.setDesiredState(frSpeed, Math.atan2(c, b));
        backLeft.setDesiredState(blSpeed, Math.atan2(d, a));
        backRight.setDesiredState(brSpeed, Math.atan2(c, a));
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
}
