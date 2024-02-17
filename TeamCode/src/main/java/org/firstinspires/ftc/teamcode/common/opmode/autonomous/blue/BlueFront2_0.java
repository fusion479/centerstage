package org.firstinspires.ftc.teamcode.common.opmode.autonomous.blue;

import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.ARM_LIFT_DELAY;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.BF_L_SPIKE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.BF_R_SPIKE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.BLUE_L_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.BLUE_M_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.BLUE_R_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.MIDDLE_SPIKE_DISTANCE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.POST_PRELOAD_WAIT;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.PRELOAD_SCORE_DELAY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants;
import org.firstinspires.ftc.teamcode.common.subsystem.Camera;
import org.firstinspires.ftc.teamcode.common.subsystem.ScoringFSM;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Blue Front 2+0", group = "_Auto")
public class BlueFront2_0 extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    SampleMecanumDrive drive;
    Camera camera = new Camera("blue");
    ScoringFSM scoringFSM = new ScoringFSM();
    private int region;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        camera.init(hardwareMap);

        TrajectorySequence leftSpikeMark = drive.trajectorySequenceBuilder(AutoConstants.BLUE_FRONT_START)
                .waitSeconds(10)
                .forward(14)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(BF_L_SPIKE, BF_L_SPIKE.getHeading())
                // END OF SPIKE MARK
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringFSM.ready();
                })
                .setTangent(Math.toRadians(130))
                .splineToLinearHeading(new Pose2d(-40, 38, Math.toRadians(0)), Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(-40, 12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(27.5, 12, Math.toRadians(0)))
                .splineToLinearHeading(BLUE_L_BACKDROP, Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(ARM_LIFT_DELAY, () -> {
                    scoringFSM.bottom();
                })
                .UNSTABLE_addTemporalMarkerOffset(PRELOAD_SCORE_DELAY, () -> {
                    scoringFSM.score();
                    scoringFSM.deposit.openInner();
                    scoringFSM.deposit.openOuter();
                })
                .waitSeconds(POST_PRELOAD_WAIT)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringFSM.ready();
                })
                .build();

        TrajectorySequence rightSpikeMark = drive.trajectorySequenceBuilder(AutoConstants.BLUE_FRONT_START)
                .waitSeconds(10)
                .forward(14)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(BF_R_SPIKE, BF_R_SPIKE.getHeading())
                // END OF SPIKE MARK
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringFSM.ready();
                })
                .setTangent(Math.toRadians(50))
                .splineToLinearHeading(new Pose2d(-34, 38, Math.toRadians(270)), Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(-34, 12, Math.toRadians(270)))
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(27.5, 12, Math.toRadians(0)))
                .splineToLinearHeading(BLUE_R_BACKDROP, Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(ARM_LIFT_DELAY, () -> {
                    scoringFSM.bottom();
                })
                .UNSTABLE_addTemporalMarkerOffset(PRELOAD_SCORE_DELAY, () -> {
                    scoringFSM.score();
                    scoringFSM.deposit.openInner();
                    scoringFSM.deposit.openOuter();
                })
                .waitSeconds(POST_PRELOAD_WAIT)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringFSM.ready();
                })
                .build();

        TrajectorySequence middleSpikeMark = drive.trajectorySequenceBuilder(AutoConstants.BLUE_FRONT_START)
                .waitSeconds(10)
                .forward(MIDDLE_SPIKE_DISTANCE)
                .back(5)
                // END OF SPIKE MARK
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringFSM.ready();
                })
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-52, 24, Math.toRadians(270)), Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(0)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(27.5, 12, Math.toRadians(0)))
                .splineToLinearHeading(BLUE_M_BACKDROP, Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(ARM_LIFT_DELAY, () -> {
                    scoringFSM.bottom();
                })
                .UNSTABLE_addTemporalMarkerOffset(PRELOAD_SCORE_DELAY, () -> {
                    scoringFSM.score();
                    scoringFSM.deposit.openInner();
                    scoringFSM.deposit.openOuter();
                })
                .waitSeconds(POST_PRELOAD_WAIT)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringFSM.ready();
                })
                .build();

        scoringFSM.init(hardwareMap);
        scoringFSM.autoInit();

        while (!isStarted() && !isStopRequested()) {
            scoringFSM.update(gamepad1, gamepad2);
            region = camera.whichRegion();
            tele.addData("DETECTED REGION", camera.whichRegion());
            tele.update();
        }

        camera.stopStreaming();

        drive.setPoseEstimate(AutoConstants.BLUE_FRONT_START);

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