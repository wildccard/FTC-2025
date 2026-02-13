package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.RobotContainer;

@Autonomous(name = "Auto Sort + Shoot", group = "Main")
public class AutoSortShoot extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotContainer robot = new RobotContainer(hardwareMap);
        robot.determineSortingPatternFromVision();

        telemetry.addData("Chosen Pattern", robot.sorting.getPattern());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Example auto flow: spin up, track, shoot if sorted.
        robot.shooter.setHoodPosition(0.6);
        robot.shooter.setFlywheelPower(1.0);

        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < 3000) {
            robot.turret.update();
            if (robot.vision.seesBlueGoalTag() || robot.vision.seesRedGoalTag()) {
                robot.turret.aimAtTagYaw(robot.vision.getGoalTagYawErrorDeg());
            }
            sleep(20);
        }

        if (robot.sorting.isSorted()) {
            robot.sorting.flickAllOrdered();
            sleep(450);
            robot.sorting.resetShovels();
        }

        robot.shooter.stop();
        robot.turret.stop();
        robot.swerve.stop();
        robot.intake.stop();
    }
}
