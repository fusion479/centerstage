package org.firstinspires.ftc.teamcode.common.opmode.auton.blue;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.subsystem.Camera;
import org.firstinspires.ftc.teamcode.common.subsystem.ScoringFSM;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.opmode.auton.AutoConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Backstage Blue", group = "_Auto")
@Config
public class BackstageBlue extends LinearOpMode {
    SampleMecanumDrive drive;
    Camera camera;
    ScoringFSM score;
    private int region;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        camera = new Camera();
        score = new ScoringFSM();

        camera.init(hardwareMap);
        score.init(hardwareMap);

        drive.setPoseEstimate(AutoConstants.BACKSTAGE_BLUE_START);

        TrajectorySequence middle = drive.trajectorySequenceBuilder(AutoConstants.BACKSTAGE_BLUE_START)
                .lineToLinearHeading(AutoConstants.CENTER_SPIKE_MARK)
                .build();

        while (opModeIsActive() && isStopRequested()) {
            region = camera.whichRegion();
            telemetry.addData("Region: ", region);
            telemetry.update();
        }
        waitForStart();
        camera.stopStreaming();
        if (region == 1) {

        } else if (region == 3) {

        } else {
            drive.followTrajectorySequenceAsync(middle);
        }

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            score.loop();

            telemetry.addData("Region: ", region);
            telemetry.update();
        }
    }
}
