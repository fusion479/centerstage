package org.firstinspires.ftc.teamcode.common.opmode.test.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.subsystem.Launcher;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Launcher Test", group = "testing")
@Config
public class LauncherTest extends LinearOpMode {
    SampleMecanumDrive drive;
    Launcher launcher = new Launcher();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        launcher.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.a) {
                launcher.idle();
            } else if (gamepad1.b) {
                launcher.launch();
            }

            drive.update();
            launcher.update();
        }
    }
}
