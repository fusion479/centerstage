package org.firstinspires.ftc.teamcode.common.opmode.testing.mechTesting;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.subsystem.ScoringFSM;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "ScoringFSM Test", group = "testing")
@Config
public class ScoringFSMTest extends LinearOpMode {
    SampleMecanumDrive drive;
    ScoringFSM scoringFSM = new ScoringFSM();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        scoringFSM.init(hardwareMap);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            if (gamepad1.a) {
                scoringFSM.intake();
            } else if (gamepad1.b) {
                scoringFSM.readyLow();
            } else if (gamepad1.x) {
                scoringFSM.readyHigh();
            } else if (gamepad1.y) {
                scoringFSM.readyMedium();
            } else if (gamepad1.right_bumper) {
                scoringFSM.score();
            }

            drive.update();
            scoringFSM.update(gamepad1);
        }
    }
}
