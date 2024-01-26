package org.firstinspires.ftc.teamcode.common.opmode.auton.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Blue Forward", group = "_Auto")
public class BlueForward extends LinearOpMode {
    SampleMecanumDrive drive;
    Deposit deposit = new Deposit();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        deposit.init(hardwareMap);

        TrajectorySequence lets_go = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(270)))
                .forward(25)
                .build();

        waitForStart();

        drive.followTrajectorySequenceAsync(lets_go);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
}
