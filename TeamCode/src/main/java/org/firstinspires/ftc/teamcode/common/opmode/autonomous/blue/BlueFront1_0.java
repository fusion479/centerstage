package org.firstinspires.ftc.teamcode.common.opmode.autonomous.blue;

import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants;
import org.firstinspires.ftc.teamcode.common.subsystem.Camera;
import org.firstinspires.ftc.teamcode.common.subsystem.ScoringFSM;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Blue Front 1+0", group = "_Auto")
public class BlueFront1_0 extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    private int region;

    SampleMecanumDrive drive;
    ScoringFSM scoringFSM = new ScoringFSM();
    Camera camera = new Camera("blue");

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        camera.init(hardwareMap);

        TrajectorySequence leftSpikeMark = drive.trajectorySequenceBuilder(AutoConstants.BLUE_FRONT_START)
                .forward(14)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(BF_L_SPIKE, BF_L_SPIKE.getHeading())
//                // END OF SPIKE MARK
//                .setTangent(Math.toRadians(130))
//                .splineToLinearHeading(new Pose2d(-40, 38, Math.toRadians(0)), Math.toRadians(270))
//                .lineToLinearHeading(new Pose2d(-40, 12, Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(40, 12, Math.toRadians(0)))
//                .lineToLinearHeading(BF_L_BACKDROP)
//
//                .UNSTABLE_addTemporalMarkerOffset(armLiftDelay, () -> {
//                    scoringFSM.low();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(preloadScoreDelay, () -> {
//                    scoringFSM.score();
//                    scoringFSM.deposit.openOuter();
//                    scoringFSM.deposit.openInner();
//                })
//                .waitSeconds(3)
//                .strafeLeft(25)
                .back(5)
                .build();

        TrajectorySequence rightSpikeMark = drive.trajectorySequenceBuilder(AutoConstants.BLUE_FRONT_START)
                .forward(14)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(BF_R_SPIKE, BF_R_SPIKE.getHeading())
                .back(5)
                // END OF SPIKE MARK
//                .setTangent(Math.toRadians(50))
//                .splineToLinearHeading(new Pose2d(-34, 38, Math.toRadians(270)), Math.toRadians(270))
//                .lineToLinearHeading(new Pose2d(-34, 12, Math.toRadians(270)))
//                .turn(Math.toRadians(90))
//                .lineToLinearHeading(new Pose2d(40, 12, Math.toRadians(0)))
//                .lineToLinearHeading(BF_R_BACKDROP)
//
//                .UNSTABLE_addTemporalMarkerOffset(armLiftDelay, () -> {
//                    scoringFSM.low();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(preloadScoreDelay, () -> {
//                    scoringFSM.score();
//                    scoringFSM.deposit.openOuter();
//                    scoringFSM.deposit.openInner();
//                })
//                .waitSeconds(3)
//                .strafeLeft(30)
                .build();

        TrajectorySequence middleSpikeMark = drive.trajectorySequenceBuilder(AutoConstants.BLUE_FRONT_START)
                .forward(MIDDLE_SPIKE_DISTANCE)
                .back(5)
                // END OF SPIKE MARK
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(-52, 24, Math.toRadians(270)), Math.toRadians(270))
//                .setTangent(Math.toRadians(270))
//                .splineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(0)), Math.toRadians(0))
//                .lineToLinearHeading(new Pose2d(40, 12, Math.toRadians(0)))
//                .lineToLinearHeading(BF_M_BACKDROP)
//                .UNSTABLE_addTemporalMarkerOffset(armLiftDelay, () -> {
//                    scoringFSM.low();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(preloadScoreDelay, () -> {
//                    scoringFSM.score();
//                    scoringFSM.deposit.openOuter();
//                    scoringFSM.deposit.openInner();
//                })
//                .waitSeconds(3)
//                .strafeLeft(27)
                .back(5)
                .build();

        scoringFSM.init(hardwareMap);
        scoringFSM.autoInit();

        while (!isStarted() && !isStopRequested()) {
            scoringFSM.update(gamepad1, gamepad2);
            region = camera.whichRegion();
            tele.addData("DETECTED REGION", camera.whichRegion());
            tele.update();
        }

        camera.stopStreaming();

        drive.setPoseEstimate(AutoConstants.BLUE_FRONT_START);

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