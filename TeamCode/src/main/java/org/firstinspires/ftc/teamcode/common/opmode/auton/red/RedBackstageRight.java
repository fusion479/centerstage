package org.firstinspires.ftc.teamcode.common.opmode.auton.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Red Backstage Right", group = "_Auto")
public class RedBackstageRight extends LinearOpMode {
    SampleMecanumDrive drive;
    Deposit deposit = new Deposit();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        deposit.init(hardwareMap);

        // .lineToLinearHeading(new Pose2d(0,-36, toRadians(180)))

        TrajectorySequence lets_go = drive.trajectorySequenceBuilder(new Pose2d(15.5, -63.25, Math.toRadians(0)))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(20, -38), Math.toRadians(180))
                .build();

        waitForStart();

        drive.followTrajectorySequenceAsync(lets_go);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
}
