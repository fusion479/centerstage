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
    public static double speedCoefficient = 0.7;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();
    SampleMecanumDrive drive;
    ScoringFSM scoringFSM = new ScoringFSM();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        scoringFSM.init(hardwareMap);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {

            if (scoringFSM.up) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * speedCoefficient,
                                -gamepad1.left_stick_x * speedCoefficient,
                                -gamepad1.right_stick_y * speedCoefficient
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

            drive.update();
            scoringFSM.update(gamepad1, gamepad2);

            telemetry.addData("State", scoringFSM.state);
            telemetry.addData("Arm timer", scoringFSM.timer.milliseconds());
            telemetry.update();
        }
    }
}
