package org.firstinspires.ftc.teamcode.common.opmode.autonomous.red;

import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.ARM_LIFT_DELAY;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.INITIAL_FORWARD_DIST;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.MIDDLE_SPIKE_DISTANCE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.POST_PRELOAD_WAIT;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.PRELOAD_SCORE_DELAY;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.RB_L_SPIKE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.RB_R_SPIKE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.RED_BACKSTAGE_START;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.RED_L_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.RED_M_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.RED_R_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.R_PARK;

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

@Autonomous(name = "Red Backstage 2+0", group = "_Auto")
public class RedBackstage2_0 extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    ElapsedTime timer = new ElapsedTime();
    SampleMecanumDrive drive;
    ScoringFSM scoringFSM = new ScoringFSM();
    Camera camera = new Camera("red");
    private int region;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scoringFSM.init(hardwareMap);
        camera.init(hardwareMap);

        drive.setPoseEstimate(RED_BACKSTAGE_START);

        TrajectorySequence leftSpikeMark = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START)
                .forward(INITIAL_FORWARD_DIST)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(RB_L_SPIKE, RB_L_SPIKE.getHeading())
                .setTangent(Math.toRadians(320))
                .splineToLinearHeading(new Pose2d(14, -38, Math.toRadians(120)), Math.toRadians(270))
                .lineToLinearHeading(RED_L_BACKDROP)
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
                .back(5)
                .lineToLinearHeading(R_PARK)
                .build();

        TrajectorySequence middleSpikeMark = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START)
                .forward(MIDDLE_SPIKE_DISTANCE)
                .back(10)
                .lineToLinearHeading(RED_M_BACKDROP)

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
                .back(5)
                .lineToLinearHeading(R_PARK)
                .build();

        TrajectorySequence rightSpikeMark = drive.trajectorySequenceBuilder(RED_BACKSTAGE_START)
                .forward(INITIAL_FORWARD_DIST)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(RB_R_SPIKE, RB_R_SPIKE.getHeading())
                .setTangent(180)
                .splineToLinearHeading(new Pose2d(12, -42, Math.toRadians(0)), Math.toRadians(90))
                .lineToLinearHeading(RED_R_BACKDROP)
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
                .back(5)
                .lineToLinearHeading(R_PARK)
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