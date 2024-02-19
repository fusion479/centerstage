package org.firstinspires.ftc.teamcode.common.opmode.autonomous.blue;

import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.ARM_LIFT_DELAY;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.CLOSE_MID;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.FRONT_INITIAL;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.FRONT_LEFT_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.FRONT_LEFT_SPIKE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.FRONT_MIDDLE_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.FRONT_PARK;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.FRONT_RIGHT_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.FRONT_RIGHT_SPIKE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.FRONT_START;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.MIDDLE_SPIKE_DISTANCE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.POST_PRELOAD_WAIT;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.PRELOAD_SCORE_DELAY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.subsystem.Camera;
import org.firstinspires.ftc.teamcode.common.subsystem.ScoringFSM;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
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

        TrajectorySequence leftSpikeMark = drive.trajectorySequenceBuilder(FRONT_START)
                .waitSeconds(5)
                .forward(14)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(FRONT_LEFT_SPIKE, FRONT_LEFT_SPIKE.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringFSM.ready();
                })
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(FRONT_INITIAL, Math.toRadians(90))
                .strafeRight(4.75)
                .lineToLinearHeading(new Pose2d(-40, 12, Math.toRadians(0)))
                .lineToLinearHeading(
                        CLOSE_MID,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL - 15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 15))
                .splineToLinearHeading(
                        FRONT_LEFT_BACKDROP,
                        Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL - 15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 15))
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
                .back(7)
                .lineToLinearHeading(FRONT_PARK)
                .build();

        TrajectorySequence rightSpikeMark = drive.trajectorySequenceBuilder(FRONT_START)
                .waitSeconds(5)
                .forward(14)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(FRONT_RIGHT_SPIKE, FRONT_RIGHT_SPIKE.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringFSM.ready();
                })
                .setTangent(Math.toRadians(10))
                .splineToLinearHeading(FRONT_INITIAL, Math.toRadians(90))
                .strafeLeft(1.25)
                .lineToLinearHeading(new Pose2d(-34, 12, Math.toRadians(270)))
                .turn(Math.toRadians(90))
                .lineToLinearHeading(
                        CLOSE_MID,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL - 15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 15))
                .splineToLinearHeading(
                        FRONT_RIGHT_BACKDROP,
                        Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL - 15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 15))
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
                .back(7)
                .lineToLinearHeading(FRONT_PARK)
                .build();

        TrajectorySequence middleSpikeMark = drive.trajectorySequenceBuilder(FRONT_START)
                .waitSeconds(5)
                .forward(MIDDLE_SPIKE_DISTANCE)
                .lineToLinearHeading(FRONT_INITIAL)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringFSM.ready();
                })
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-52, 24, Math.toRadians(270)), Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(0)), Math.toRadians(0))
                .lineToLinearHeading(
                        CLOSE_MID,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL - 15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 15))
                .splineToLinearHeading(
                        FRONT_MIDDLE_BACKDROP,
                        Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL - 15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 15))
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
                .back(7)
                .lineToLinearHeading(FRONT_PARK)
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

        drive.setPoseEstimate(FRONT_START);

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