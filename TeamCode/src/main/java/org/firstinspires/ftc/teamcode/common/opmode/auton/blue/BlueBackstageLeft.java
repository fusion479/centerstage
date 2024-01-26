package org.firstinspires.ftc.teamcode.common.opmode.auton.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.opmode.auton.AutoConstants;
import org.firstinspires.ftc.teamcode.common.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Blue Backstage Left", group = "_Auto")
public class BlueBackstageLeft extends LinearOpMode {
    SampleMecanumDrive drive;
    Deposit deposit = new Deposit();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        deposit.init(hardwareMap);

        TrajectorySequence lets_go = drive.trajectorySequenceBuilder(AutoConstants.BLUE_BACKSTAGE_START)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(16, 40, Math.toRadians(330)), Math.toRadians(0))
                .build();

        drive.setPoseEstimate(AutoConstants.BLUE_BACKSTAGE_START);

        waitForStart();

        drive.followTrajectorySequenceAsync(lets_go);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
}
