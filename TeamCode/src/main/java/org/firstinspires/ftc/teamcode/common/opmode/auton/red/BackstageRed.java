package org.firstinspires.ftc.teamcode.common.opmode.auton.red;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.opmode.auton.AutoConstants;
import org.firstinspires.ftc.teamcode.common.subsystem.Camera;
import org.firstinspires.ftc.teamcode.common.subsystem.ScoringFSM;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Backstage Red", group = "_Auto")
@Config
public class BackstageRed extends LinearOpMode {
    SampleMecanumDrive drive;
    Camera camera;
    ScoringFSM score;
    private int region;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        camera = new Camera();


        camera.init(hardwareMap);
        score.init(hardwareMap);

        drive.setPoseEstimate(AutoConstants.BACKSTAGE_BLUE_START);

        TrajectorySequence left = drive.trajectorySequenceBuilder(AutoConstants.BACKSTAGE_BLUE_START)
                .splineTo(AutoConstants.LEFT_SPIKE_MARK, 0)
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(AutoConstants.BACKSTAGE_BLUE_START)
                .forward(28)
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(AutoConstants.BACKSTAGE_BLUE_START)
                .splineTo(AutoConstants.RIGHT_SPIKE_MARK, 0)
                .build();

        while (opModeIsActive() && isStopRequested()) {
            region = camera.whichRegion();
            telemetry.addData("Region: ", region);
            telemetry.update();
        }
        waitForStart();
        camera.stopStreaming();
//        if (region == 1) {
            drive.followTrajectorySequenceAsync(left);

//        } else if (region == 3) {
//            drive.followTrajectorySequenceAsync(right);

//        } else {
//            drive.followTrajectorySequenceAsync(middle);
//        }


        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            score.update();

            telemetry.addData("Region: ", region);
            telemetry.update();
        }
    }
}
