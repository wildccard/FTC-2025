package org.firstinspires.ftc.teamcode.util;

/**
 * Abstracts heading source (Pinpoint/odometry/IMU) for subsystems that need field awareness.
 */
public interface HeadingProvider {
    /** @return robot heading in radians, increasing CCW. */
    double getHeadingRadians();
}
