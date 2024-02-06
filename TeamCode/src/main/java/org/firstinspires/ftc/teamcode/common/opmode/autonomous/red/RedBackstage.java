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
    Camera camera = new Camera("red");

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scoringFSM.init(hardwareMap);
        camera.init(hardwareMap);

        drive.setPoseEstimate(RED_BACKSTAGE_START);

        TrajectorySequence leftSpikeMark = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START)
//                TODO: OLD PATH, TEST NEW ONE
//                .forward(INITIAL_FORWARD_DIST)
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(RB_L_SPIKE, RB_L_SPIKE.getHeading())
//                .lineToLinearHeading(RB_L_BACKDROP)
//                .forward(RB_PRELOAD_FORWARD_DIST)
                .forward(INITIAL_FORWARD_DIST)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(RB_L_SPIKE, RB_L_SPIKE.getHeading())
                .setTangent(Math.toRadians(320))
                .splineToLinearHeading(new Pose2d(14, -38, Math.toRadians(120)), Math.toRadians(270))
                .lineToLinearHeading(RB_L_BACKDROP)

                .UNSTABLE_addTemporalMarkerOffset(armLiftDelay, () -> {
                    scoringFSM.low();
                })
                .UNSTABLE_addTemporalMarkerOffset(preloadScoreDelay, () -> {
                    scoringFSM.score();
                    scoringFSM.deposit.openInner();
                    scoringFSM.deposit.openOuter();
                })

                .waitSeconds(postPreloadWait)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringFSM.ready();
                })
                .back(3)
                .lineToLinearHeading(new Pose2d(49, -60, Math.toRadians(0)))
                .build();

        TrajectorySequence middleSpikeMark = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START)
//                .forward(MIDDLE_SPIKE_DISTANCE)
//                .back(10)
//                .lineToLinearHeading(RB_M_BACKDROP)
//                .forward(RB_PRELOAD_FORWARD_DIST)

                .forward(MIDDLE_SPIKE_DISTANCE)
                .back(10)
                .lineToLinearHeading(RB_M_BACKDROP)

                .UNSTABLE_addTemporalMarkerOffset(armLiftDelay, () -> {
                    scoringFSM.low();
                })
                .UNSTABLE_addTemporalMarkerOffset(preloadScoreDelay, () -> {
                    scoringFSM.score();
                    scoringFSM.deposit.openInner();
                    scoringFSM.deposit.openOuter();
                })
                .waitSeconds(postPreloadWait)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringFSM.ready();
                })
                .back(3)
                .lineToLinearHeading(new Pose2d(49, -60, Math.toRadians(0)))
                .build();

        TrajectorySequence rightSpikeMark = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START)
//                .forward(14)
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(RB_R_SPIKE, RB_R_SPIKE.getHeading())
//                .lineToLinearHeading(RB_R_BACKDROP)
//                .forward(RB_PRELOAD_FORWARD_DIST)

                .forward(INITIAL_FORWARD_DIST)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(RB_R_SPIKE, RB_R_SPIKE.getHeading())
                .setTangent(180)
                .splineToLinearHeading(new Pose2d(12, -42, Math.toRadians(0)), Math.toRadians(90))
                .lineToLinearHeading(RB_R_BACKDROP)

                .UNSTABLE_addTemporalMarkerOffset(armLiftDelay, () -> {
                    scoringFSM.low();
                })
                .UNSTABLE_addTemporalMarkerOffset(preloadScoreDelay, () -> {
                    scoringFSM.score();
                    scoringFSM.deposit.openInner();
                    scoringFSM.deposit.openOuter();
                })

                .waitSeconds(postPreloadWait)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringFSM.ready();
                })
                .back(3)
                .lineToLinearHeading(new Pose2d(49, -60, Math.toRadians(0)))
                .build();



        timer.reset();
        scoringFSM.autoInit();

        while (!isStarted() && !isStopRequested()) {
            scoringFSM.update(gamepad1, gamepad2);
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
            scoringFSM.update(gamepad1, gamepad2);
            drive.update();
        }
    }
}