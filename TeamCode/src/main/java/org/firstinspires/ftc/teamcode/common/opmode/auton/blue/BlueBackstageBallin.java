package org.firstinspires.ftc.teamcode.common.opmode.auton.blue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.opmode.auton.AutoConstants;
import org.firstinspires.ftc.teamcode.common.subsystem.Arm;
import org.firstinspires.ftc.teamcode.common.subsystem.Camera;
import org.firstinspires.ftc.teamcode.common.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystem.Intake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Blue Backstage BALLIN", group = "_Auto")
public class BlueBackstageBallin extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();
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

        arm.ready();
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
                .forward(25)
                .back(10)
                .build();


        while (!isStarted() && !isStopRequested()) {
            region = camera.whichRegion();
            telemetry.addData("Region", camera.whichRegion());
            telemetry.update();
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
