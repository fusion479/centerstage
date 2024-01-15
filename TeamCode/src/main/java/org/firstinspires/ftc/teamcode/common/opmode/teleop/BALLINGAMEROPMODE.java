package org.firstinspires.ftc.teamcode.common.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.subsystem.Arm;
import org.firstinspires.ftc.teamcode.common.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystem.Intake;
import org.firstinspires.ftc.teamcode.common.subsystem.Launcher;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "BALLINGAMEROPMODE", group = "!!!")
public class BALLINGAMEROPMODE extends LinearOpMode {
    SampleMecanumDrive drive;
    Arm arm = new Arm();
    Deposit deposit = new Deposit();
    Intake intake = new Intake();
    Launcher launcher = new Launcher();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        arm.init(hardwareMap);
        deposit.init(hardwareMap);
        intake.init(hardwareMap);
        launcher.init(hardwareMap);

        intake.intaking();



        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            if (gamepad1.a) {
                arm.down();
                intake.intaking();
                deposit.accepting();
            } else if (gamepad1.x) {
                arm.up();
                deposit.ready();
            } else if (gamepad1.y) {
                arm.up();
                deposit.score();
                deposit.toggleOuter();
            } else if (gamepad1.b) {
                arm.up();
                deposit.score();
            }


            if (gamepad1.right_trigger > 0.1) {
                intake.setPower(gamepad1.right_trigger * .8);

            } else if (gamepad1.left_trigger > 0.1) {
                intake.setPower(-gamepad1.left_trigger * .8);

            } else{
                intake.setPower(0);
            }

            intake.update();


            arm.update();
            deposit.update();
            drive.update();
        }
    }
}