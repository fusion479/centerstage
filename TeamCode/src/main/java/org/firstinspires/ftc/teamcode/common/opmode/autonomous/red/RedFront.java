package org.firstinspires.ftc.teamcode.common.opmode.autonomous.red;

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
    Camera camera = new Camera();
    ScoringFSM scoringFSM = new ScoringFSM();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        camera.init(hardwareMap);

        // TODO: LEFT AND RIGHT SPIKE MARK ARE NOT THE CORRECT PATH
        TrajectorySequence leftSpikeMark = drive.trajectorySequenceBuilder(AutoConstants.RED_FRONT_START)
                .forward(14)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-40, -39, Math.toRadians(140)), Math.toRadians(140))
                .back(5)
                .build();

        TrajectorySequence rightSpikeMark = drive.trajectorySequenceBuilder(AutoConstants.RED_FRONT_START)
                .forward(14)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-32, -39, Math.toRadians(50)), Math.toRadians(50))
                .back(5)
                .build();

        TrajectorySequence middleSpikeMark = drive.trajectorySequenceBuilder(AutoConstants.RED_FRONT_START)
                .forward(AutoConstants.MIDDLE_SPIKE_DISTANCE)
                .back(10)
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
