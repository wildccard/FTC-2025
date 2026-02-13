package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RobotContainer;

@TeleOp(name = "Main TeleOp Swerve", group = "Main")
public class MainTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotContainer robot = new RobotContainer(hardwareMap);

        telemetry.addLine("Init done");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            robot.turret.update();

            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double omega = -gamepad1.right_stick_x;
            robot.swerve.drive(x, y, omega, true);

            if (gamepad1.right_trigger > 0.05) {
                robot.intake.run(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.05) {
                robot.intake.run(-gamepad1.left_trigger);
            } else {
                robot.intake.stop();
            }

            if (gamepad2.a) {
                robot.shooter.setFlywheelPower(1.0);
            } else if (gamepad2.b) {
                robot.shooter.stop();
            }

            if (gamepad2.dpad_up) robot.shooter.setHoodPosition(0.75);
            if (gamepad2.dpad_left) robot.shooter.setHoodPosition(0.5);
            if (gamepad2.dpad_down) robot.shooter.setHoodPosition(0.25);

            boolean trackBlue = gamepad2.left_bumper && robot.vision.seesBlueGoalTag();
            boolean trackRed = gamepad2.right_bumper && robot.vision.seesRedGoalTag();
            if (trackBlue || trackRed) {
                robot.turret.aimAtTagYaw(robot.vision.getGoalTagYawErrorDeg());
            } else {
                robot.turret.stop();
            }

            if (gamepad2.x && robot.sorting.isSorted()) {
                robot.sorting.flickAllOrdered();
            }
            if (gamepad2.y) {
                robot.sorting.resetShovels();
            }

            telemetry.addData("Sorted", robot.sorting.isSorted());
            telemetry.addData("Pattern", robot.sorting.getPattern());
            telemetry.addData("Turret(rad)", robot.turret.getTurretAngleRad());
            telemetry.update();
        }

        robot.swerve.stop();
        robot.intake.stop();
        robot.shooter.stop();
        robot.turret.stop();
    }
}
