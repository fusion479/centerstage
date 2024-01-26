package org.firstinspires.ftc.teamcode.common.opmode.auton.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Blue Front Right", group = "_Auto")
public class BlueFrontRight extends LinearOpMode {
    SampleMecanumDrive drive;
    Deposit deposit = new Deposit();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        deposit.init(hardwareMap);

        TrajectorySequence lets_go = drive.trajectorySequenceBuilder(new Pose2d(-34.5, 63.25, Math.toRadians(270)))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-27, 42), Math.toRadians(90))
                .build();

        waitForStart();

        drive.followTrajectorySequenceAsync(lets_go);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
}
