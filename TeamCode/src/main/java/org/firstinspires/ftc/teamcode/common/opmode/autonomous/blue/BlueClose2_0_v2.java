package org.firstinspires.ftc.teamcode.common.opmode.autonomous.blue;

import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.ARM_LIFT_DELAY;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.CLOSE_INITIAL;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.CLOSE_LEFT_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.CLOSE_LEFT_SPIKE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.CLOSE_MIDDLE_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.CLOSE_PARK;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.CLOSE_RIGHT_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.CLOSE_RIGHT_SPIKE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.CLOSE_START;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.INITIAL_FORWARD_DIST;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.MIDDLE_SPIKE_DISTANCE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.POST_PRELOAD_WAIT;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.PRELOAD_SCORE_DELAY;

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

@Autonomous(name = "Blue Close 2+0 Apriltag", group = "_Auto")
public class BlueClose2_0_v2 extends LinearOpMode {
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
                .lineToLinearHeading(CLOSE_MIDDLE_BACKDROP)
                .back(16)
                .build();

        TrajectorySequence leftSpikeMark = drive.trajectorySequenceBuilder(CLOSE_START)
                .forward(INITIAL_FORWARD_DIST)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(CLOSE_LEFT_SPIKE, CLOSE_LEFT_SPIKE.getHeading())
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(CLOSE_INITIAL, Math.toRadians(90))
                .back(7)
                .lineToLinearHeading(CLOSE_LEFT_BACKDROP)
                .back(16)
                .build();

        TrajectorySequence rightSpikeMark = drive.trajectorySequenceBuilder(CLOSE_START)
                .forward(INITIAL_FORWARD_DIST)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(CLOSE_RIGHT_SPIKE, CLOSE_RIGHT_SPIKE.getHeading())
                .setTangent(Math.toRadians(30))
                .splineToLinearHeading(CLOSE_INITIAL, Math.toRadians(90))
                .lineToLinearHeading(CLOSE_RIGHT_BACKDROP)
                .back(16)
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
        camera.setManualExposure(6, 250, isStopRequested(), telemetry, this);

        while (opModeIsActive() && !isStopRequested()) {
            switch (autoState) {
                case SPIKE_MARK:
                    drive.update();
                    if (!drive.isBusy()) {
                        autoState = STATES.APRIL_TAG;
                        timer.reset();
                    }
                    break;
                case APRIL_TAG:
                    if (camera.detectAprilTag(telemetry)) {
                        camera.moveRobot(drive, telemetry);
                    }

                    if (timer.milliseconds() >= 3500) {
                        autoState = STATES.BACKDROP_SCORE;
                        drive.setPoseEstimate(drive.getPoseEstimate());
                        TrajectorySequence middleSpikeMarkAfter = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .forward(5)
                                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                                    scoringFSM.bottom();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
                                    scoringFSM.score();
                                    scoringFSM.deposit.openOuter();
                                    scoringFSM.deposit.openInner();
                                })
                                .waitSeconds(3)
                                .back(10)
                                .lineToLinearHeading(CLOSE_PARK)
                                .build();
                        drive.followTrajectorySequenceAsync(middleSpikeMarkAfter);
                    }
                    break;
                case BACKDROP_SCORE:
                    drive.update();
                    if (!drive.isBusy()) {
                        autoState = STATES.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }

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
        IDLE
    }
}
