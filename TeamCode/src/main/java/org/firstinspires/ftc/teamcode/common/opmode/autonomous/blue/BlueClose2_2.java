package org.firstinspires.ftc.teamcode.common.opmode.autonomous.blue;

import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.FancyPrint;
import org.firstinspires.ftc.teamcode.common.subsystem.Camera;
import org.firstinspires.ftc.teamcode.common.subsystem.ScoringFSM;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Blue Close 2+2", group = "_Auto")
public class BlueClose2_2 extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime loopTime = new ElapsedTime();
    SampleMecanumDrive drive;
    ScoringFSM scoringFSM = new ScoringFSM();
    Camera camera = new Camera("blue");
    private int region;
    private STATES autoState;
    private int scoreCounter = 0;

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
                .lineToLinearHeading(MIDDLE_BACKDROP)
                .build();

        TrajectorySequence leftSpikeMark = drive.trajectorySequenceBuilder(CLOSE_START)
                .forward(INITIAL_FORWARD_DIST)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(CLOSE_LEFT_SPIKE, CLOSE_LEFT_SPIKE.getHeading())
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(CLOSE_INITIAL, Math.toRadians(90))
                .back(7)
                .lineToLinearHeading(LEFT_BACKDROP)
                .build();

        TrajectorySequence rightSpikeMark = drive.trajectorySequenceBuilder(CLOSE_START)
                .forward(INITIAL_FORWARD_DIST)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(CLOSE_RIGHT_SPIKE, CLOSE_RIGHT_SPIKE.getHeading())
                .setTangent(Math.toRadians(30))
                .splineToLinearHeading(CLOSE_INITIAL, Math.toRadians(90))
                .lineToLinearHeading(RIGHT_BACKDROP)
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
                        drive.setMotorPowers(0, 0, 0, 0);
                    }

                    if (timer.milliseconds() >= 2000) {
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
                        drive.setPoseEstimate(drive.getPoseEstimate());
                        if (scoreCounter < 1) {
                            autoState = STATES.BACKDROP_TO_STACK;
                            TrajectorySequence backdropToStack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .waitSeconds(POST_PRELOAD_WAIT)
                                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                                        scoringFSM.ready();
                                    })
                                    .lineToLinearHeading(BACKDROP_TRUSS_ENTRANCE)
                                    .lineToLinearHeading(WING_TRUSS_ENTRANCE)
                                    .lineToLinearHeading(STACK_1)
                                    .build();
                            drive.followTrajectorySequenceAsync(backdropToStack);
                            scoreCounter++;
                            timer.reset();
                        } else {
                            autoState = STATES.PARK;
                            TrajectorySequence park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .back(10)
                                    .lineToLinearHeading(CLOSE_PARK)
                                    .build();
                            drive.followTrajectorySequenceAsync(park);
                        }
                    }
                    break;
                case BACKDROP_TO_STACK:
                    if (!drive.isBusy()) {
                        drive.setPoseEstimate(drive.getPoseEstimate());
                        autoState =  STATES.STACK_TO_BACKDROP;
                        TrajectorySequence stackToBackdrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(WING_TRUSS_ENTRANCE)
                                .lineToLinearHeading(BACKDROP_TRUSS_ENTRANCE)
                                .lineToLinearHeading(MIDDLE_BACKDROP)
                                .build();
                        drive.followTrajectorySequenceAsync(stackToBackdrop);
                    }
                    break;
                case STACK_TO_BACKDROP:
                    if (!drive.isBusy()) {
                        camera.setDesiredTag(2);
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
        BACKDROP_TO_STACK,
        STACK_TO_BACKDROP,
        PARK,
        IDLE
    }
}
