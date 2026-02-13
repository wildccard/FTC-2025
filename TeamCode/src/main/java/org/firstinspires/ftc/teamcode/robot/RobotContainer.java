package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointHeadingProvider;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SortingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwerveModule;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.SortingPattern;

/**
 * Centralized robot construction so TeleOp and Auto share identical subsystem wiring.
 */
public class RobotContainer {
    public final SwerveDriveSubsystem swerve;
    public final IntakeSubsystem intake;
    public final SortingSubsystem sorting;
    public final TurretSubsystem turret;
    public final ShooterSubsystem shooter;
    public final VisionSubsystem vision;

    public RobotContainer(HardwareMap hw) {
        PinpointHeadingProvider headingProvider = new PinpointHeadingProvider();

        SwerveModule fl = new SwerveModule(
                hw.get(DcMotorEx.class, "fl_drive"),
                hw.get(CRServo.class, "fl_steer"),
                hw.get(AnalogInput.class, "fl_encoder"),
                0.0);
        SwerveModule fr = new SwerveModule(
                hw.get(DcMotorEx.class, "fr_drive"),
                hw.get(CRServo.class, "fr_steer"),
                hw.get(AnalogInput.class, "fr_encoder"),
                0.0);
        SwerveModule bl = new SwerveModule(
                hw.get(DcMotorEx.class, "bl_drive"),
                hw.get(CRServo.class, "bl_steer"),
                hw.get(AnalogInput.class, "bl_encoder"),
                0.0);
        SwerveModule br = new SwerveModule(
                hw.get(DcMotorEx.class, "br_drive"),
                hw.get(CRServo.class, "br_steer"),
                hw.get(AnalogInput.class, "br_encoder"),
                0.0);

        swerve = new SwerveDriveSubsystem(fl, fr, bl, br, headingProvider, 1.0, 1.0);

        intake = new IntakeSubsystem(
                hw.get(DcMotorEx.class, "intake_left"),
                hw.get(DcMotorEx.class, "intake_right"));

        sorting = new SortingSubsystem(
                hw.get(ColorSensor.class, "slot1_a"), hw.get(ColorSensor.class, "slot1_b"), hw.get(Servo.class, "shovel1"),
                hw.get(ColorSensor.class, "slot2_a"), hw.get(ColorSensor.class, "slot2_b"), hw.get(Servo.class, "shovel2"),
                hw.get(ColorSensor.class, "slot3_a"), hw.get(ColorSensor.class, "slot3_b"), hw.get(Servo.class, "shovel3"));

        turret = new TurretSubsystem(
                hw.get(CRServo.class, "turret_left"),
                hw.get(CRServo.class, "turret_right"),
                hw.get(DcMotorEx.class, "turret_encoder"),
                headingProvider);

        shooter = new ShooterSubsystem(
                hw.get(DcMotorEx.class, "flywheel_left"),
                hw.get(DcMotorEx.class, "flywheel_right"),
                hw.get(Servo.class, "hood"));

        vision = new VisionSubsystem(hw);
    }

    public void determineSortingPatternFromVision() {
        SortingPattern pattern = vision.scanSortingPattern();
        sorting.setPattern(pattern);
    }
}
