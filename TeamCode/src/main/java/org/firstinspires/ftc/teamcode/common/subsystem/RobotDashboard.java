package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class RobotDashboard extends Mechanism {
    public static double speedCoefficient = 1;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();
    SampleMecanumDrive drive;
    ScoringFSM scoringFSM = new ScoringFSM();
    Launcher launcher = new Launcher();

    @Override
    public void init(HardwareMap hwMap) {
        drive = new SampleMecanumDrive(hwMap);
        scoringFSM.init(hwMap);
        launcher.init(hwMap);

        scoringFSM.ready();
        launcher.idle();
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        if (scoringFSM.up) {
            speedCoefficient = .45;
        } else {
            speedCoefficient = 1;
        }

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * speedCoefficient,
                        -gamepad1.left_stick_x * speedCoefficient,
                        -gamepad1.right_stick_y * speedCoefficient
                )
        );

        if (gamepad2.x) {
            launcher.launch();
        } else if (gamepad2.a) {
            launcher.idle();
        }

        drive.update();
        scoringFSM.update(gamepad1, gamepad2);
        launcher.update();

//        telemetry.addData("State", scoringFSM.state);
//        telemetry.addData("Arm timer", scoringFSM.timer.milliseconds());
//        telemetry.addData("isPressedA", scoringFSM.isPressedA);
//        telemetry.addData("A", gamepad1.a);
//        telemetry.addData("Inner locked", scoringFSM.deposit.innerLocked);
//        telemetry.addData("Outer locked", scoringFSM.deposit.outerLocked);
//        telemetry.update();
    }

}
