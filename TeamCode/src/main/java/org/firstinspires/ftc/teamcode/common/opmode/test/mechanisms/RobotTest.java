package org.firstinspires.ftc.teamcode.common.opmode.test.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.subsystem.Arm;
import org.firstinspires.ftc.teamcode.common.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystem.Intake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp (name = "Robot Test")
@Config
public class RobotTest extends LinearOpMode {

    SampleMecanumDrive drive;
    Deposit deposit = new Deposit();
    Arm arm = new Arm();
    Intake intake = new Intake();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        deposit.init(hardwareMap);
        arm.init(hardwareMap);
        intake.init(hardwareMap);

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
                arm.up();
            } else if (gamepad1.b) {
                arm.down();
            } else if (gamepad1.x) {
                intake.down();
            }else if (gamepad1.y) {
                intake.idle();
            } else if (gamepad1.dpad_up) {
                deposit.score();
            } else if(gamepad1.dpad_down) {
                deposit.accepting();
            }

            if (gamepad1.right_trigger > 0.1) {
                intake.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                intake.setPower(-gamepad1.left_trigger);
            } else {
                intake.setPower(0);
            }

            intake.update();
            arm.update();
            deposit.update();
            drive.update();
        }
    }
}
