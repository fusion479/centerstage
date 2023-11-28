package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Main", group = "_1")
public class Main extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // run on init
        // initialize all your hardware

        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {


        }
    }
}
