package org.firstinspires.ftc.teamcode.common.opmode.autonomous.blue;

import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.CLOSE_MID;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.FRONT_INITIAL;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.FRONT_LEFT_SPIKE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.FRONT_PARK;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.FRONT_RIGHT_SPIKE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.FRONT_START;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.LEFT_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.MIDDLE_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.MIDDLE_SPIKE_DISTANCE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.POST_PRELOAD_WAIT;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.PRELOAD_SCORE_DELAY;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.RIGHT_BACKDROP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.subsystem.Camera;
import org.firstinspires.ftc.teamcode.common.subsystem.ScoringFSM;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Blue Far 2+0", group = "_Auto")
public class BlueFar2_0 extends LinearOpMode {
    private final ElapsedTime timer = new ElapsedTime();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    SampleMecanumDrive drive;
    Camera camera = new Camera("blue");
    ScoringFSM scoringFSM = new ScoringFSM();
    private int region;
    private STATES autoState;

    @Override
    public void runOpMode() throws InterruptedException {
        autoState = STATES.SPIKE_MARK;
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
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL - 10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 10))
                .lineToLinearHeading(
                        LEFT_BACKDROP,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL - 10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 10))
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
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL - 10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 10))
                .lineToLinearHeading(
                        RIGHT_BACKDROP,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL - 10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 10))
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
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL - 10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 10))
                .lineToLinearHeading(
                        MIDDLE_BACKDROP,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL - 10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - 10))
                .build();

        scoringFSM.init(hardwareMap);
        scoringFSM.autoInit();

        while (!isStarted() && !isStopRequested()) {
            scoringFSM.update(gamepad1, gamepad2);
            region = camera.whichRegion();
            tele.addData("DETECTED REGION", camera.whichRegion());
            tele.update();
        }

        drive.setPoseEstimate(FRONT_START);

        if (region == 1) {
            drive.followTrajectorySequenceAsync(leftSpikeMark);
        } else if (region == 2) {
            drive.followTrajectorySequenceAsync(middleSpikeMark);
        } else {
            drive.followTrajectorySequenceAsync(rightSpikeMark);
        }
        camera.stopStreaming();
        camera.aprilTagInit(hardwareMap, region);
        camera.setManualExposure(6, 250, isStopRequested(), tele, this);

        while (opModeIsActive() && !isStopRequested()) {
            switch (autoState) {
                case SPIKE_MARK:
                    if (!drive.isBusy()) {
                        autoState = STATES.APRILTAG;
                        timer.reset();
                    }
                    break;
                case PARK:
                    if (!drive.isBusy()) {
                        autoState = STATES.IDLE;
                        timer.reset();
                    }
                    break;
                case APRILTAG:
                    if (camera.detectAprilTag(tele)) {
                        camera.moveRobot(drive, tele);
                    } else {
                        drive.setMotorPowers(0, 0, 0, 0);
                    }

                    if (timer.milliseconds() >= 2500) {
                        autoState = STATES.BACKDROP_SCORE;
                        drive.setPoseEstimate(drive.getPoseEstimate());
                        TrajectorySequence backdropScore = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    scoringFSM.bottom();
                                })
                                .forward(6)
                                .UNSTABLE_addTemporalMarkerOffset(PRELOAD_SCORE_DELAY, () -> {
                                    scoringFSM.score();
                                    scoringFSM.deposit.openOuter();
                                    scoringFSM.deposit.openInner();
                                })
                                .build();
                        drive.followTrajectorySequenceAsync(backdropScore);
                        timer.reset();
                    }
                    break;
                case BACKDROP_SCORE:
                    if (!drive.isBusy()) {
                        autoState = STATES.PARK;
                        drive.setPoseEstimate(drive.getPoseEstimate());
                        TrajectorySequence park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .waitSeconds(POST_PRELOAD_WAIT)
                                .back(7)
                                .lineToLinearHeading(FRONT_PARK)
                                .build();
                        drive.followTrajectorySequenceAsync(park);
                        timer.reset();
                    }
                case IDLE:
                    break;
            }

            scoringFSM.update(gamepad1, gamepad2);
            drive.update();
        }
    }

    private enum STATES {
        SPIKE_MARK,
        BACKDROP_SCORE,
        APRILTAG,
        PARK,
        IDLE,
    }
}