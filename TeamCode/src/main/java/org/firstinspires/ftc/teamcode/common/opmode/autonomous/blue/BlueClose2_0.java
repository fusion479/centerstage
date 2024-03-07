package org.firstinspires.ftc.teamcode.common.opmode.autonomous.blue;

import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.APRILTAG_TIMEOUT;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.CLOSE_INITIAL;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.CLOSE_LEFT_SPIKE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.CLOSE_PARK;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.CLOSE_RIGHT_SPIKE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.CLOSE_START;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.INITIAL_FORWARD_DIST;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.LEFT_BACKDROP_PRE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.MIDDLE_BACKDROP_PRE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.MIDDLE_SPIKE_DISTANCE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.POST_APRILTAG_FORWARD;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.POST_PRELOAD_WAIT;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.PRELOAD_SCORE_DELAY;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.RIGHT_BACKDROP_PRE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private int region;
    private STATES autoState;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scoringFSM.init(hardwareMap);
        camera.init(hardwareMap);
        drive.setPoseEstimate(CLOSE_START);

        TrajectorySequence middleSpikeMark = drive.trajectorySequenceBuilder(CLOSE_START)
                .forward(MIDDLE_SPIKE_DISTANCE)
                .back(10)
                .lineToLinearHeading(MIDDLE_BACKDROP_PRE)
                .build();

        TrajectorySequence leftSpikeMark = drive.trajectorySequenceBuilder(CLOSE_START)
                .forward(INITIAL_FORWARD_DIST)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(CLOSE_LEFT_SPIKE, CLOSE_LEFT_SPIKE.getHeading())
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(CLOSE_INITIAL, Math.toRadians(90))
                .lineToLinearHeading(LEFT_BACKDROP_PRE)

//                .splineToSplineHeading(CLOSE_LEFT_SPIKE, CLOSE_LEFT_SPIKE.getHeading())
//                .setReversed(true)
//                .splineToSplineHeading(CLOSE_INITIAL, Math.toRadians(90))
//                .splineToConstantHeading(LEFT_BACKDROP_PRE.vec(), Math.toRadians(0))
                .build();

        TrajectorySequence rightSpikeMark = drive.trajectorySequenceBuilder(CLOSE_START)
                .forward(INITIAL_FORWARD_DIST)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(CLOSE_RIGHT_SPIKE, CLOSE_RIGHT_SPIKE.getHeading())
                .setTangent(Math.toRadians(30))
                .splineToLinearHeading(CLOSE_INITIAL, Math.toRadians(90))
                .lineToLinearHeading(RIGHT_BACKDROP_PRE)

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
                        camera.relocalize(drive);
                    } else {
                        drive.setMotorPowers(0, 0, 0, 0);
                    }

                    if (timer.milliseconds() >= APRILTAG_TIMEOUT) {
                        autoState = STATES.BACKDROP_SCORE;
                        drive.setPoseEstimate(drive.getPoseEstimate());
                        TrajectorySequence backdropScore = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    scoringFSM.bottom();
                                })
                                .forward(POST_APRILTAG_FORWARD)
                                .UNSTABLE_addTemporalMarkerOffset(PRELOAD_SCORE_DELAY, () -> {
                                    scoringFSM.score();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(PRELOAD_SCORE_DELAY + 1.5, () -> {
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
                                .waitSeconds(POST_PRELOAD_WAIT)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    scoringFSM.ready();
                                })
                                .back(7)
                                .lineToLinearHeading(CLOSE_PARK)
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
