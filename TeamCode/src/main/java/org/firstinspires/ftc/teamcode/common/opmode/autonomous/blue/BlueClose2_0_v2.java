package org.firstinspires.ftc.teamcode.common.opmode.autonomous.blue;

import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.ARM_LIFT_DELAY;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.CLOSE_MIDDLE_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.CLOSE_PARK;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.CLOSE_START;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.MIDDLE_SPIKE_DISTANCE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.POST_PRELOAD_WAIT;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.PRELOAD_SCORE_DELAY;

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

@Autonomous(name = "Blue Close 2+0 Apriltag", group = "_Auto")
public class BlueClose2_0_v2 extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    ElapsedTime timer = new ElapsedTime();
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
//        camera.setManualExposure(6, 250, isStopRequested(), telemetry, this);
        drive.setPoseEstimate(CLOSE_START);

        TrajectorySequence middleSpikeMark = drive.trajectorySequenceBuilder(CLOSE_START)
                .forward(MIDDLE_SPIKE_DISTANCE)
                .back(10)
                .lineToLinearHeading(CLOSE_MIDDLE_BACKDROP)
                .back(10)
                .build();

        TrajectorySequence middleSpikeMarkAfter = drive.trajectorySequenceBuilder(middleSpikeMark.end())
                .UNSTABLE_addTemporalMarkerOffset(ARM_LIFT_DELAY, () -> {
                    scoringFSM.bottom();
                })
                .UNSTABLE_addTemporalMarkerOffset(PRELOAD_SCORE_DELAY, () -> {
                    scoringFSM.score();
                    scoringFSM.deposit.openOuter();
                    scoringFSM.deposit.openInner();
                })
                .waitSeconds(POST_PRELOAD_WAIT)
                .back(7)
                .lineToLinearHeading(CLOSE_PARK)
                .build();

        timer.reset();
        scoringFSM.autoInit();

        while (!isStarted() && !isStopRequested()) {
            scoringFSM.update(gamepad1, gamepad2);
//            region = camera.whichRegion();
            tele.addData("score timer", scoringFSM.timer.milliseconds());
//            tele.addData("DETECTED REGION", camera.whichRegion());
            tele.update();
        }

        autoState = STATES.SPIKE_MARK;
        drive.followTrajectorySequenceAsync(middleSpikeMark);

        while (opModeIsActive()) {
            switch (autoState) {
                case SPIKE_MARK:
                    if (!drive.isBusy()) {
                        autoState = STATES.BACKDROP_SCORE;
                        drive.followTrajectorySequenceAsync(middleSpikeMarkAfter);
                    }
                    break;
                case APRIL_TAG:
//                    if (camera.detectAprilTag(telemetry)) {
//                        camera.moveRobot(drive);
//                    }
//                    if (timer.milliseconds() >= 5000) {
//                        autoState = STATES.BACKDROP_SCORE;
//                    }
                    break;
                case BACKDROP_SCORE:
                    if (!drive.isBusy()) {
                        autoState = STATES.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }

            tele.addData("current state:", autoState);

            drive.update();
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
