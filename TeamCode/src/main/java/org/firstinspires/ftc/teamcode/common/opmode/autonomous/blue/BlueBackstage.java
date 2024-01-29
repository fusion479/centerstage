package org.firstinspires.ftc.teamcode.common.opmode.autonomous.blue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants;
import org.firstinspires.ftc.teamcode.common.subsystem.Arm;
import org.firstinspires.ftc.teamcode.common.subsystem.BlueCamera;
import org.firstinspires.ftc.teamcode.common.subsystem.Camera;
import org.firstinspires.ftc.teamcode.common.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystem.Intake;
import org.firstinspires.ftc.teamcode.common.subsystem.Lift;
import org.firstinspires.ftc.teamcode.common.subsystem.ScoringFSM;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Blue Backstage", group = "_Auto")
public class BlueBackstage extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    private int region;

    ElapsedTime timer = new ElapsedTime();

    SampleMecanumDrive drive;
    Lift lift = new Lift();
    Arm arm = new Arm();
    Deposit deposit = new Deposit();
    Intake intake = new Intake();
    Camera camera = new Camera();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.init(hardwareMap);
        arm.init(hardwareMap);
        deposit.init(hardwareMap);
        intake.init(hardwareMap);
        camera.init(hardwareMap);

        drive.setPoseEstimate(AutoConstants.RED_BACKSTAGE_START);

        TrajectorySequence leftSpikeMark = drive.trajectorySequenceBuilder(AutoConstants.BLUE_BACKSTAGE_START)
                .forward(14)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(16, 39, Math.toRadians(310)), Math.toRadians(310))
                .lineToLinearHeading(new Pose2d(12, 44, Math.toRadians(0)))
                .forward(32)
                .build();

        TrajectorySequence rightSpikeMark = drive.trajectorySequenceBuilder(AutoConstants.BLUE_BACKSTAGE_START)
                .forward(14)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(8, 39, Math.toRadians(230)), Math.toRadians(230))
                .back(5)
                .build();

        TrajectorySequence middleSpikeMark = drive.trajectorySequenceBuilder(AutoConstants.BLUE_BACKSTAGE_START)
                .forward(AutoConstants.MIDDLE_SPIKE_DISTANCE)
                .back(5)
                .build();

        timer.reset();

        while (!isStarted() && !isStopRequested()) {
            lift.bottom();
            arm.autoInit();
            deposit.idle();
            deposit.lockOuter();
            deposit.lockInner();

            if (timer.milliseconds() > 2750) {
                intake.up();
            }

            lift.update();
            arm.update();
            deposit.update();
            intake.update();
            region = camera.whichRegion();
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
            lift.update();
            arm.update();
            deposit.update();
            intake.update();
            drive.update();
        }
    }
}
