package org.firstinspires.ftc.teamcode.common.opmode.testing.mechTesting;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.subsystem.Arm;
import org.firstinspires.ftc.teamcode.common.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystem.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp (name = "Robot Test")
@Config
public class RobotTest extends LinearOpMode {

    SampleMecanumDrive drive;
    Lift lift = new Lift();
    Arm arm = new Arm();
    Deposit deposit = new Deposit();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        lift.init(hardwareMap);
        arm.init(hardwareMap);
        deposit.init(hardwareMap);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            if (gamepad1.a) {
                arm.down();
                deposit.intake();
            } else if (gamepad1.b) {
                arm.down();
                deposit.idle();
            } else if (gamepad1.x) {
                arm.up();
                deposit.ready();
            } else if (gamepad1.y) {
                arm.up();
                deposit.score();
            }

            lift.loop();
            arm.loop();
            deposit.loop();
            drive.update();
        }
    }
}
