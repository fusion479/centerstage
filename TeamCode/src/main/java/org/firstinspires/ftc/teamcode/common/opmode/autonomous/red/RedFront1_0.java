package org.firstinspires.ftc.teamcode.common.opmode.autonomous.red;

import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.FRONT_LEFT_SPIKE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.FRONT_RIGHT_SPIKE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.FRONT_START;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.MIDDLE_SPIKE_DISTANCE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.reflectY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.subsystem.Camera;
import org.firstinspires.ftc.teamcode.common.subsystem.ScoringFSM;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Red Front 1+0", group = "_Auto")
public class RedFront1_0 extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    SampleMecanumDrive drive;
    Camera camera = new Camera("red");
    ScoringFSM scoringFSM = new ScoringFSM();
    private int region;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        camera.init(hardwareMap);

        // TODO: LEFT AND RIGHT SPIKE MARK ARE NOT THE CORRECT PATH
        TrajectorySequence leftSpikeMark = drive.trajectorySequenceBuilder(FRONT_START)
                .forward(14)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(reflectY(FRONT_LEFT_SPIKE), reflectY(FRONT_LEFT_SPIKE).getHeading())
                .back(5)
                .build();

        TrajectorySequence rightSpikeMark = drive.trajectorySequenceBuilder(FRONT_START)
                .forward(14)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(reflectY(FRONT_RIGHT_SPIKE), reflectY(FRONT_RIGHT_SPIKE).getHeading())
                .back(5)
                .build();

        TrajectorySequence middleSpikeMark = drive.trajectorySequenceBuilder(FRONT_START)
                .forward(MIDDLE_SPIKE_DISTANCE)
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

        drive.setPoseEstimate(reflectY(FRONT_START));

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
