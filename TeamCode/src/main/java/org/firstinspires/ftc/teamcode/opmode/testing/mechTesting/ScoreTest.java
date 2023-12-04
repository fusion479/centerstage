package org.firstinspires.ftc.teamcode.opmode.testing.mechTesting;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.subsystem.ScoringFSM;

@TeleOp(name = "ScoreFSM Test", group = "testing")
@Config
public class ScoreTest extends LinearOpMode {
    SampleMecanumDrive drive;
    ScoringFSM scoringFSM = new ScoringFSM();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        scoringFSM.init(hardwareMap);

        waitForStart();

        // Scan servo till stop pressed.
        while(opModeIsActive()){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();
            scoringFSM.loop();
        }
    }
}
