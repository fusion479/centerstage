package org.firstinspires.ftc.teamcode.common.opmode.autonomous.blue;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants;
import org.firstinspires.ftc.teamcode.common.subsystem.Camera;
import org.firstinspires.ftc.teamcode.common.subsystem.ScoringFSM;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Blue Close 2+2", group = "_Auto")
public class BlueClose2_2 extends LinearOpMode {
    private static final double STARTING_ANGLE = 270; // + makes angle narrower (hits truss)     - makes angle steeper (hits wall)
    private static final double VEL_OFFSET = 20;
    private static final double ACCEL_OFFSET = 30;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime loopTime = new ElapsedTime();
    SampleMecanumDrive drive;
    ScoringFSM scoringFSM = new ScoringFSM();
    Camera camera = new Camera("blue");
    AutoConstants constants;
    private int region;
    private STATES autoState;
    private int scoreCounter = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        constants = new AutoConstants();
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scoringFSM.init(hardwareMap);
        camera.init(hardwareMap);
        drive.setPoseEstimate(constants.CLOSE_START);

        TrajectorySequence middleSpikeMark = drive.trajectorySequenceBuilder(constants.CLOSE_START)
                .forward(constants.MIDDLE_SPIKE_DISTANCE)
                .back(10)
                .lineToLinearHeading(constants.MIDDLE_BACKDROP_PRE)
                .build();

        TrajectorySequence leftSpikeMark = drive.trajectorySequenceBuilder(constants.CLOSE_START)
                .forward(constants.INITIAL_FORWARD_DIST)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(constants.CLOSE_LEFT_SPIKE, constants.CLOSE_LEFT_SPIKE.getHeading())
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(constants.CLOSE_INITIAL, Math.toRadians(90))
                .lineToLinearHeading(constants.LEFT_BACKDROP_PRE)
                .build();

        TrajectorySequence rightSpikeMark = drive.trajectorySequenceBuilder(constants.CLOSE_START)
                .forward(constants.INITIAL_FORWARD_DIST)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(constants.CLOSE_RIGHT_SPIKE, constants.CLOSE_RIGHT_SPIKE.getHeading())
                .setTangent(Math.toRadians(30))
                .splineToLinearHeading(constants.CLOSE_INITIAL, Math.toRadians(90))
                .lineToLinearHeading(constants.RIGHT_BACKDROP_PRE)
                .build();

        timer.reset();
        scoringFSM.autoInit();

        while (!isStarted() && !isStopRequested()) {
            scoringFSM.update(gamepad1, gamepad2);
            region = camera.whichRegion();
            tele.addData("score timer", scoringFSM.timer.milliseconds());
            tele.addData("DETECTED REGION", region);
            tele.update();
        }

        autoState = STATES.SPIKE_MARK;
        if (region == 1) {
            drive.followTrajectorySequenceAsync(leftSpikeMark);
        } else if (region == 3) {
            drive.followTrajectorySequenceAsync(rightSpikeMark);
        } else {
            drive.followTrajectorySequenceAsync(middleSpikeMark);
        }
        camera.stopStreaming();
        camera.aprilTagInit(hardwareMap, region);
        camera.setManualExposure(6, 250, isStopRequested(), tele, this);

        while (opModeIsActive() && !isStopRequested()) {
            switch (autoState) {
                case SPIKE_MARK:
                    if (!drive.isBusy()) {
                        autoState = STATES.APRIL_TAG;
                        timer.reset();
                    }
                    break;
                case APRIL_TAG:
                    if (camera.detectAprilTag(tele)) {
                        camera.moveRobot(drive, tele);
                    } else {
                        camera.relocalize(drive);
                        drive.setMotorPowers(0, 0, 0, 0);
                    }

                    if (timer.milliseconds() >= constants.APRILTAG_TIMEOUT) {
                        autoState = STATES.BACKDROP_SCORE;
                        drive.setPoseEstimate(drive.getPoseEstimate());
                        if (scoreCounter == 0) {
                            TrajectorySequence backdropScore = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                        scoringFSM.bottom();
                                    })
                                    .forward(constants.POST_APRILTAG_FORWARD)
                                    .UNSTABLE_addTemporalMarkerOffset(constants.PRELOAD_SCORE_DELAY, () -> {
                                        scoringFSM.score();
                                    })
                                    .UNSTABLE_addTemporalMarkerOffset(constants.PRELOAD_SCORE_DELAY + 2, () -> {
                                        scoringFSM.deposit.openOuter();
                                        scoringFSM.deposit.openInner();
                                    })
                                    .waitSeconds(0.5)
                                    .build();
                            drive.followTrajectorySequenceAsync(backdropScore);
                        } else if (scoreCounter == 1) {
                            TrajectorySequence backdropScore2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                        scoringFSM.low();
                                    })
                                    .forward(constants.POST_APRILTAG_FORWARD)
                                    .UNSTABLE_addTemporalMarkerOffset(constants.PRELOAD_SCORE_DELAY, () -> {
                                        scoringFSM.score();
                                    })
                                    .UNSTABLE_addTemporalMarkerOffset(constants.PRELOAD_SCORE_DELAY + 2, () -> {
                                        scoringFSM.deposit.openOuter();
                                        scoringFSM.deposit.openInner();
                                    })
                                    .waitSeconds(0.5)
                                    .build();
                            drive.followTrajectorySequenceAsync(backdropScore2);
                        }
                        timer.reset();
                    }
                    break;
                case BACKDROP_SCORE:
                    if (!drive.isBusy()) {
                        drive.setPoseEstimate(drive.getPoseEstimate());
                        if (scoreCounter < 1) {
                            autoState = STATES.BACKDROP_TO_STACK;
                            TrajectorySequence backdropToStack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .waitSeconds(constants.POST_PRELOAD_WAIT)
                                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                        scoringFSM.ready();
                                    })
                                    .lineToLinearHeading(constants.MIDDLE_BACKDROP_PRE,
                                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL - VEL_OFFSET, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - ACCEL_OFFSET))
                                    .setTangent(Math.toRadians(STARTING_ANGLE))
                                    .splineToLinearHeading(
                                            new Pose2d(30, 11, Math.toRadians(0)),
                                            Math.toRadians(180),
                                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL - VEL_OFFSET, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - ACCEL_OFFSET))
                                    .lineToLinearHeading(new Pose2d(-55, 11))

                                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                        scoringFSM.stack();
                                    })
                                    .waitSeconds(constants.STACK_PICKUP_DELAY)
                                    .build();
                            drive.followTrajectorySequenceAsync(backdropToStack);
                            scoreCounter++;
                            timer.reset();
                        } else {
                            autoState = STATES.PARK;
                            TrajectorySequence park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .waitSeconds(constants.POST_PRELOAD_WAIT)
                                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                        scoringFSM.ready();
                                    })
                                    .back(10)
                                    .lineToLinearHeading(constants.CLOSE_PARK)
                                    .build();
                            drive.followTrajectorySequenceAsync(park);
                        }
                    }
                    break;
                case BACKDROP_TO_STACK:
                    if (!drive.isBusy()) {
                        drive.setPoseEstimate(drive.getPoseEstimate());
                        autoState = STATES.STACK_TO_BACKDROP;
                        TrajectorySequence stackToBackdrop = drive.trajectorySequenceBuilder(new Pose2d(-55, 11, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(30, 11))
                                .splineToLinearHeading(
                                        constants.MIDDLE_BACKDROP_PRE,
                                        Math.toRadians(STARTING_ANGLE + 180),
                                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL - VEL_OFFSET, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - ACCEL_OFFSET))
                                .build();
                        drive.followTrajectorySequenceAsync(stackToBackdrop);
                        timer.reset();
                    }
                    break;
                case STACK_TO_BACKDROP:
                    if (!drive.isBusy()) {
                        autoState = STATES.APRIL_TAG;
                        timer.reset();
                    }
                    break;
                case PARK:
                    if (!drive.isBusy()) {
                        autoState = STATES.IDLE;
                        timer.reset();
                    }
                    break;
                case IDLE:
                    break;
            }

            drive.update();
            tele.addData("camera finished", camera.getFinished());
            tele.addData("score counter", scoreCounter);
            tele.addData("current state", autoState);
            tele.addData("detected region", region);
            tele.addData("loop time", loopTime.milliseconds());
            loopTime.reset();
            tele.update();

            scoringFSM.update(gamepad1, gamepad2);
        }
    }

    private enum STATES {
        SPIKE_MARK,
        APRIL_TAG,
        BACKDROP_SCORE,
        BACKDROP_SCORE_2,
        BACKDROP_TO_STACK,
        STACK_TO_BACKDROP,
        PARK,
        IDLE
    }
}
