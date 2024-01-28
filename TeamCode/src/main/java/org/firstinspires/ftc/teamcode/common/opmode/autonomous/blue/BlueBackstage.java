package org.firstinspires.ftc.teamcode.common.opmode.autonomous.blue;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants;
import org.firstinspires.ftc.teamcode.common.subsystem.Arm;
import org.firstinspires.ftc.teamcode.common.subsystem.Camera;
import org.firstinspires.ftc.teamcode.common.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystem.Intake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Blue Backstage", group = "_Auto")
public class BlueBackstage extends LinearOpMode {
    MultipleTelemetry tele = new MultipleTelemetry();
    private int region;

    SampleMecanumDrive drive;
    Deposit deposit = new Deposit();
    Intake intake = new Intake();
    Arm arm = new Arm();
    Camera camera = new Camera();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        deposit.init(hardwareMap);
        intake.init(hardwareMap);
        arm.init(hardwareMap);
        camera.init(hardwareMap);

        arm.autoInit();
        deposit.idle();
        intake.up();

        arm.update();
        deposit.update();
        intake.update();

        drive.setPoseEstimate(AutoConstants.BLUE_BACKSTAGE_START);

        TrajectorySequence leftSpikeMark = drive.trajectorySequenceBuilder(AutoConstants.BLUE_BACKSTAGE_START)
                .strafeLeft(3)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(16, 32, Math.toRadians(330)), Math.toRadians(300))
                .back(5)
                .build();

        TrajectorySequence rightSpikeMark = drive.trajectorySequenceBuilder(AutoConstants.BLUE_BACKSTAGE_START)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(10, 40, Math.toRadians(210)), Math.toRadians(180))
                .back(10)
                .build();

        TrajectorySequence middleSpikeMark = drive.trajectorySequenceBuilder(AutoConstants.BLUE_BACKSTAGE_START)
                .forward(AutoConstants.MIDDLE_SPIKE_DISTANCE)
                .back(10)
                .build();


        while (!isStarted() && !isStopRequested()) {
            region = camera.whichRegion();
            tele.addData("DETECTED REGION", camera.whichRegion());
            tele.update();
        }

        camera.stopStreaming();

        if (region == 1) {
            drive.followTrajectorySequenceAsync(rightSpikeMark);
        } else if (region == 2) {
            drive.followTrajectorySequenceAsync(middleSpikeMark);
        } else {
            drive.followTrajectorySequenceAsync(leftSpikeMark);
        }

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
}
