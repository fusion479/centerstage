package org.firstinspires.ftc.teamcode.common.opmode.test.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.subsystem.ScoringFSM;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "ScoringFSM Test", group = "testing")
@Config
public class ScoringFSMTest extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();

    SampleMecanumDrive drive;
    ScoringFSM scoringFSM = new ScoringFSM();

    public static double speedCoefficient = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        scoringFSM.init(hardwareMap);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()){

            if (scoringFSM.up) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * speedCoefficient,
                                -gamepad1.left_stick_x * speedCoefficient,
                                -gamepad1.right_stick_x * speedCoefficient
                        )
                );
            } else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
            }

            if (gamepad1.a) {
                scoringFSM.intake();
            } else if (gamepad1.b) {
                scoringFSM.readyBottom();
            } else if (gamepad1.x) {
                scoringFSM.readyHigh();
            } else if (gamepad1.y) {
                scoringFSM.readyMedium();
            } else if (gamepad1.left_bumper || gamepad1.right_bumper) {
                scoringFSM.score();
            }

            drive.update();
            scoringFSM.update(gamepad1);

            telemetry.addData("State", scoringFSM.state);
            telemetry.addData("Arm timer", scoringFSM.armTimer.milliseconds());
            telemetry.update();
        }
    }
}
