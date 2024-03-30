package org.firstinspires.ftc.teamcode.common.opmode.autonomous.blue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants;
import org.firstinspires.ftc.teamcode.common.subsystem.Camera;
import org.firstinspires.ftc.teamcode.common.subsystem.ScoringFSM;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Blue Close 2+0", group = "_Auto")
public class BlueClose2_0 extends LinearOpMode {
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

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scoringFSM.init(hardwareMap);
        camera.init(hardwareMap);
        constants = new AutoConstants();
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

//                .splineToSplineHeading(CLOSE_LEFT_SPIKE, CLOSE_LEFT_SPIKE.getHeading())
//                .setReversed(true)
//                .splineToSplineHeading(CLOSE_INITIAL, Math.toRadians(90))
//                .splineToConstantHeading(LEFT_BACKDROP_PRE.vec(), Math.toRadians(0))
                .build();

        TrajectorySequence rightSpikeMark = drive.trajectorySequenceBuilder(constants.CLOSE_START)
                .forward(constants.INITIAL_FORWARD_DIST)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(constants.CLOSE_RIGHT_SPIKE, constants.CLOSE_RIGHT_SPIKE.getHeading())
                .setTangent(Math.toRadians(30))
                .splineToLinearHeading(constants.CLOSE_INITIAL, Math.toRadians(90))
                .lineToLinearHeading(constants.RIGHT_BACKDROP_PRE)

//                .forward(10)
//                .splineToSplineHeading(CLOSE_RIGHT_SPIKE, CLOSE_RIGHT_SPIKE.getHeading())
//                .setReversed(true)
//                .splineToSplineHeading(CLOSE_INITIAL, Math.toRadians(90))
//                .splineToConstantHeading(RIGHT_BACKDROP.vec(), Math.toRadians(0))
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
//        camera.setManualExposure(6, 250, isStopRequested(), tele, this);

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
                        drive.setMotorPowers(0, 0, 0, 0);
                            camera.relocalize(drive);
                    }

                    if (timer.milliseconds() >= constants.APRILTAG_TIMEOUT) {
                        autoState = STATES.BACKDROP_SCORE;
                        drive.setPoseEstimate(drive.getPoseEstimate());
                        TrajectorySequence backdropScore = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    scoringFSM.bottom();
                                })
                                .forward(constants.POST_APRILTAG_FORWARD)
                                .UNSTABLE_addTemporalMarkerOffset(constants.PRELOAD_SCORE_DELAY, () -> {
                                    scoringFSM.score();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(constants.PRELOAD_SCORE_DELAY + 1, () -> {
                                    scoringFSM.deposit.openOuter();
                                    scoringFSM.deposit.openInner();
                                })
                                .waitSeconds(0.5)
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
                                .waitSeconds(constants.POST_PRELOAD_WAIT)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    scoringFSM.ready();
                                })
                                .back(7)
                                .lineToLinearHeading(constants.CLOSE_PARK)
                                .build();
                        drive.followTrajectorySequenceAsync(park);
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
        PARK,
        IDLE
    }
}
