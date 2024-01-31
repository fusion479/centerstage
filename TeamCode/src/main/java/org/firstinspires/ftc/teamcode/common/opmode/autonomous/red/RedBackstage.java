package org.firstinspires.ftc.teamcode.common.opmode.autonomous.red;

import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.common.subsystem.Camera;
import org.firstinspires.ftc.teamcode.common.subsystem.ScoringFSM;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Red Backstage", group = "_Auto")
public class RedBackstage extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    private int region;

    ElapsedTime timer = new ElapsedTime();

    SampleMecanumDrive drive;
    ScoringFSM scoringFSM = new ScoringFSM();
    Camera camera = new Camera();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scoringFSM.init(hardwareMap);
        camera.init(hardwareMap);

        drive.setPoseEstimate(RED_BACKSTAGE_START);

        TrajectorySequence leftSpikeMark = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START)
                .forward(INITIAL_FORWARD_DIST)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(RB_L_SPIKE, RB_L_SPIKE.getHeading())
                .lineToLinearHeading(RB_L_BACKDROP)
                .forward(RB_PRELOAD_FORWARD_DIST)
                .UNSTABLE_addTemporalMarkerOffset(armLiftDelay, () -> {
                    scoringFSM.bottom();
                })
                .UNSTABLE_addTemporalMarkerOffset(preloadScoreDelay, () -> {
                    scoringFSM.score();
                    scoringFSM.deposit.openInner();
                    scoringFSM.deposit.openOuter();
                })
                .back(10)
                .waitSeconds(postPreloadWait)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringFSM.ready();
                })
                .forward(5)
                .build();

        TrajectorySequence middleSpikeMark = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START)
                .forward(MIDDLE_SPIKE_DISTANCE)
                .back(10)
                .lineToLinearHeading(RB_L_BACKDROP)
                .forward(RB_PRELOAD_FORWARD_DIST)
                .UNSTABLE_addTemporalMarkerOffset(armLiftDelay, () -> {
                    scoringFSM.bottom();
                })
                .UNSTABLE_addTemporalMarkerOffset(preloadScoreDelay, () -> {
                    scoringFSM.score();
                    scoringFSM.deposit.openInner();
                    scoringFSM.deposit.openOuter();
                })
                .waitSeconds(postPreloadWait)
                .back(10)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringFSM.ready();
                })
                .forward(5)
                .build();

        TrajectorySequence rightSpikeMark = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START)
                .forward(14)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(RB_R_SPIKE, RB_R_SPIKE.getHeading())
                .lineToLinearHeading(RB_R_BACKDROP)
                .forward(RB_PRELOAD_FORWARD_DIST)
                .UNSTABLE_addTemporalMarkerOffset(armLiftDelay, () -> {
                    scoringFSM.bottom();
                })
                .UNSTABLE_addTemporalMarkerOffset(preloadScoreDelay, () -> {
                    scoringFSM.score();
                    scoringFSM.deposit.openInner();
                    scoringFSM.deposit.openOuter();
                })

                .waitSeconds(postPreloadWait)
                .back(10)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringFSM.ready();
                })
                .forward(5)
                .build();



        timer.reset();
        scoringFSM.autoInit();

        while (!isStarted() && !isStopRequested()) {
            scoringFSM.update(gamepad1);
            region = camera.whichRegion();
            tele.addData("score timer", scoringFSM.timer.milliseconds());
            tele.addData("DETECTED REGION", camera.whichRegion());
            tele.update();
        }

        camera.stopStreaming();

        if (region == 1) {
            drive.followTrajectorySequenceAsync(leftSpikeMark);
        } else if (region == 2) {
            drive.followTrajectorySequenceAsync(middleSpikeMark);
        } else {
            drive.followTrajectorySequenceAsync(rightSpikeMark);
        }

        while (opModeIsActive() && !isStopRequested()) {
            scoringFSM.update(gamepad1);
            drive.update();
        }
    }
}