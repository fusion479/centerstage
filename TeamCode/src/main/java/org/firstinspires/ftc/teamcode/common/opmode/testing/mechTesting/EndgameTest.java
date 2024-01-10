package org.firstinspires.ftc.teamcode.common.opmode.testing.mechTesting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.subsystem.Launcher;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Endgame Test", group = "testing")
@Config
public class EndgameTest extends LinearOpMode {
    SampleMecanumDrive drive;
//    Climber climber = new Climber();
    Launcher launcher = new Launcher();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
//        climber.init(hardwareMap);
        launcher.init(hardwareMap);

        waitForStart();

        // Scan servo till stop pressed.
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
