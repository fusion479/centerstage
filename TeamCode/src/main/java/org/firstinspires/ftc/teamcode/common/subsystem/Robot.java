package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Robot extends Mechanism {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();

    SampleMecanumDrive drive;
    ScoringFSM scoringFSM = new ScoringFSM();

    public static double speedCoefficient = 0.7;

    @Override
    public void init(HardwareMap hwMap) {
        drive = new SampleMecanumDrive(hwMap);
        scoringFSM.init(hwMap);
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        if (scoringFSM.up) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * speedCoefficient,
                            -gamepad1.left_stick_x * speedCoefficient,
                            gamepad1.right_stick_x * speedCoefficient
                    )
            );
        } else {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            gamepad1.right_stick_x
                    )
            );
        }

        drive.update();
        scoringFSM.update(gamepad1);

        telemetry.addData("State", scoringFSM.state);
        telemetry.addData("Arm timer", scoringFSM.timer.milliseconds());
        telemetry.addData("isPressedA", scoringFSM.isPressedA);
        telemetry.addData("A", gamepad1.a);
        telemetry.addData("Inner locked", scoringFSM.deposit.innerLocked);
        telemetry.addData("Outer locked", scoringFSM.deposit.outerLocked);
        telemetry.update();
    }

}
