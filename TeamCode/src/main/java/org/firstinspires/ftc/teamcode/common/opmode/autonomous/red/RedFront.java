package org.firstinspires.ftc.teamcode.common.opmode.autonomous.red;

import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants;
import org.firstinspires.ftc.teamcode.common.subsystem.Arm;
import org.firstinspires.ftc.teamcode.common.subsystem.Camera;
import org.firstinspires.ftc.teamcode.common.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystem.Intake;
import org.firstinspires.ftc.teamcode.common.subsystem.ScoringFSM;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Red Front", group = "_Auto")
public class RedFront extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    private int region;

    SampleMecanumDrive drive;
    Camera camera = new Camera("red");
    ScoringFSM scoringFSM = new ScoringFSM();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        camera.init(hardwareMap);

        // TODO: LEFT AND RIGHT SPIKE MARK ARE NOT THE CORRECT PATH
        TrajectorySequence leftSpikeMark = drive.trajectorySequenceBuilder(AutoConstants.RED_FRONT_START)
                .forward(14)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(RF_L_SPIKE, RF_L_SPIKE.getHeading())
                // END OF SPIKE MARK
                .setTangent(Math.toRadians(320))
                .splineToLinearHeading(new Pose2d(-34, -38, Math.toRadians(90)), Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(-34, -12, Math.toRadians(90)))
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(40, -12, Math.toRadians(0)))
                .lineToLinearHeading(RF_L_BACKDROP)
                .UNSTABLE_addTemporalMarkerOffset(armLiftDelay, () -> {
                    scoringFSM.low();
                })
                .UNSTABLE_addTemporalMarkerOffset(preloadScoreDelay, () -> {
                    scoringFSM.score();
                    scoringFSM.deposit.openInner();
                    scoringFSM.deposit.openOuter();
                })

                .waitSeconds(postPreloadWait)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringFSM.ready();
                })
                .strafeRight(30)
                .build();

        TrajectorySequence rightSpikeMark = drive.trajectorySequenceBuilder(AutoConstants.RED_FRONT_START)
                .forward(14)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(RF_R_SPIKE, Math.toRadians(50))
                // END OF SPIKE MARK
                .setTangent(Math.toRadians(230))
                .splineToLinearHeading(new Pose2d(-40, -38, Math.toRadians(0)), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-40, -12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(40, -12, Math.toRadians(0)))
                .lineToLinearHeading(RF_R_BACKDROP)
                .UNSTABLE_addTemporalMarkerOffset(armLiftDelay, () -> {
                    scoringFSM.low();
                })
                .UNSTABLE_addTemporalMarkerOffset(preloadScoreDelay, () -> {
                    scoringFSM.score();
                    scoringFSM.deposit.openInner();
                    scoringFSM.deposit.openOuter();
                })

                .waitSeconds(postPreloadWait)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringFSM.ready();
                })
                .strafeRight(25)
                .build();

        TrajectorySequence middleSpikeMark = drive.trajectorySequenceBuilder(AutoConstants.RED_FRONT_START)
                .forward(AutoConstants.MIDDLE_SPIKE_DISTANCE)
                .back(5)
                // END OF SPIKE MARK
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-52, -24, Math.toRadians(90)), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(0)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(40, -12, Math.toRadians(0)))
                .lineToLinearHeading(RF_M_BACKDROP)
                .UNSTABLE_addTemporalMarkerOffset(armLiftDelay, () -> {
                    scoringFSM.low();
                })
                .UNSTABLE_addTemporalMarkerOffset(preloadScoreDelay, () -> {
                    scoringFSM.score();
                    scoringFSM.deposit.openInner();
                    scoringFSM.deposit.openOuter();
                })

                .waitSeconds(postPreloadWait)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringFSM.ready();
                })
                .strafeRight(27)
                .build();

        scoringFSM.init(hardwareMap);
        scoringFSM.autoInit();

        while (!isStarted() && !isStopRequested()) {
            scoringFSM.update(gamepad1);
            region = camera.whichRegion();
            tele.addData("DETECTED REGION", camera.whichRegion());
            tele.update();
        }

        camera.stopStreaming();

        drive.setPoseEstimate(AutoConstants.RED_FRONT_START);

        if (region == 1) {
            drive.followTrajectorySequenceAsync(leftSpikeMark);
        } else if (region == 2) {
            drive.followTrajectorySequenceAsync(middleSpikeMark);
        } else {
            drive.followTrajectorySequenceAsync(rightSpikeMark);
        }

        while (opModeIsActive() && !isStopRequested()) {
            scoringFSM.update(gamepad1);
            drive.update();
        }
    }
}
