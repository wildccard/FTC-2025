package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.SortingPattern;

/**
 * Thin wrapper around Limelight A1 AprilTag readings.
 *
 * Replace getLatestTag() internals with your actual Limelight SDK call.
 */
public class VisionSubsystem {
    public VisionSubsystem(HardwareMap hardwareMap) {
        // TODO: initialize Limelight device from hardwareMap when SDK class is available.
    }

    public int getLatestTag() {
        // TODO: return detected AprilTag ID from Limelight.
        return -1;
    }

    public SortingPattern scanSortingPattern() {
        int tag = getLatestTag();
        return SortingPattern.fromTagId(tag);
    }

    public boolean seesBlueGoalTag() {
        return getLatestTag() == 20;
    }

    public boolean seesRedGoalTag() {
        return getLatestTag() == 24;
    }

    /** Horizontal error from tag center; + means target is to the right. */
    public double getGoalTagYawErrorDeg() {
        // TODO: use Limelight tx equivalent.
        return 0.0;
    }
}
