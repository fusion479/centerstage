package org.firstinspires.ftc.teamcode.common.opmode.autonomous.red;

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

@Autonomous(name = "Red Front 1+0", group = "_Auto")
public class RedFront1_0 extends LinearOpMode {
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
                .back(5)
                .lineToLinearHeading(RED_FRONT_START)
                .build();

        TrajectorySequence rightSpikeMark = drive.trajectorySequenceBuilder(AutoConstants.RED_FRONT_START)
                .forward(14)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(RF_R_SPIKE, Math.toRadians(50))
                .back(5)
                .lineToLinearHeading(RED_FRONT_START)
                .build();

        TrajectorySequence middleSpikeMark = drive.trajectorySequenceBuilder(AutoConstants.RED_FRONT_START)
                .forward(AutoConstants.MIDDLE_SPIKE_DISTANCE)
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

        drive.setPoseEstimate(AutoConstants.RED_FRONT_START);

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
