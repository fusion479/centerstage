package org.firstinspires.ftc.teamcode.opmode.testing.mechTesting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Lift;

@TeleOp(name = "Lift Test", group = "testing")
@Config
public class LiftTest extends LinearOpMode {
    Lift lift = new Lift();

    @Override
    public void runOpMode() throws InterruptedException {
        lift.init(hardwareMap);

        waitForStart();

        // Scan servo till stop pressed.
        while(opModeIsActive()){
            lift.loop();
        }
    }
}
