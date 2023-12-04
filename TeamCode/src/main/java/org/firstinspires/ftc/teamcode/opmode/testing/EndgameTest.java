package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Climber;
import org.firstinspires.ftc.teamcode.subsystem.Launcher;
import org.firstinspires.ftc.teamcode.subsystem.Lift;

@TeleOp(name = "Endgame Test", group = "testing")
@Config
public class EndgameTest extends LinearOpMode {
    SampleMecanumDrive drive;
//    Climber climber = new Climber();
    Launcher launcher = new Launcher();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        climber.init(hardwareMap);
        launcher.init(hardwareMap);

        waitForStart();

        // Scan servo till stop pressed.
        while(opModeIsActive()){
            if (gamepad1.a) {
                launcher.idle();
            } else if (gamepad1.b) {
                launcher.launch();
            }

            drive.update();
            launcher.loop();
        }
    }
}
