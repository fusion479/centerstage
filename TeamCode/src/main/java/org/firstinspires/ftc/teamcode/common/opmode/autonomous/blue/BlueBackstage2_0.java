package org.firstinspires.ftc.teamcode.common.opmode.autonomous.blue;

import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.ARM_LIFT_DELAY;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.BB_L_SPIKE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.BB_R_SPIKE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.BLUE_BACKSTAGE_START;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.BLUE_L_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.BLUE_M_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.BLUE_R_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.B_PARK;
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

import org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants;
import org.firstinspires.ftc.teamcode.common.subsystem.Camera;
import org.firstinspires.ftc.teamcode.common.subsystem.ScoringFSM;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Blue Backstage 2+0", group = "_Auto")
public class BlueBackstage2_0 extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    ElapsedTime timer = new ElapsedTime();
    SampleMecanumDrive drive;
    ScoringFSM scoringFSM = new ScoringFSM();
    Camera camera = new Camera("blue");
    private int region;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scoringFSM.init(hardwareMap);
        camera.init(hardwareMap);

        drive.setPoseEstimate(BLUE_BACKSTAGE_START);

        TrajectorySequence leftSpikeMark = drive.trajectorySequenceBuilder(AutoConstants.BLUE_BACKSTAGE_START)
                .forward(INITIAL_FORWARD_DIST)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(BB_L_SPIKE, BB_L_SPIKE.getHeading())
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(12, 42, Math.toRadians(0)), Math.toRadians(90))
                .lineToLinearHeading(BLUE_L_BACKDROP)
                .UNSTABLE_addTemporalMarkerOffset(ARM_LIFT_DELAY, () -> {
                    scoringFSM.bottom();
                })
                .UNSTABLE_addTemporalMarkerOffset(PRELOAD_SCORE_DELAY, () -> {
                    scoringFSM.score();
                    scoringFSM.deposit.openOuter();
                    scoringFSM.deposit.openInner();
                })
                .waitSeconds(POST_PRELOAD_WAIT)
                .back(5)
                .lineToLinearHeading(B_PARK)
                .build();

        TrajectorySequence middleSpikeMark = drive.trajectorySequenceBuilder(AutoConstants.BLUE_BACKSTAGE_START)
                .forward(MIDDLE_SPIKE_DISTANCE)
                .back(10)
                .lineToLinearHeading(BLUE_M_BACKDROP)

                .UNSTABLE_addTemporalMarkerOffset(ARM_LIFT_DELAY, () -> {
                    scoringFSM.bottom();
                })
                .UNSTABLE_addTemporalMarkerOffset(PRELOAD_SCORE_DELAY, () -> {
                    scoringFSM.score();
                    scoringFSM.deposit.openOuter();
                    scoringFSM.deposit.openInner();
                })
                .waitSeconds(POST_PRELOAD_WAIT)
                .back(5)
                .lineToLinearHeading(B_PARK)
                .build();

        TrajectorySequence rightSpikeMark = drive.trajectorySequenceBuilder(AutoConstants.BLUE_BACKSTAGE_START)
                .forward(INITIAL_FORWARD_DIST)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(BB_R_SPIKE, BB_R_SPIKE.getHeading())
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(14, 38, Math.toRadians(270)), Math.toRadians(0))
                .lineToLinearHeading(BLUE_R_BACKDROP)

                .UNSTABLE_addTemporalMarkerOffset(ARM_LIFT_DELAY, () -> {
                    scoringFSM.bottom();
                })
                .UNSTABLE_addTemporalMarkerOffset(PRELOAD_SCORE_DELAY, () -> {
                    scoringFSM.score();
                    scoringFSM.deposit.openOuter();
                    scoringFSM.deposit.openInner();
                })
                .waitSeconds(POST_PRELOAD_WAIT)
                .back(5)
                .lineToLinearHeading(B_PARK)
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
