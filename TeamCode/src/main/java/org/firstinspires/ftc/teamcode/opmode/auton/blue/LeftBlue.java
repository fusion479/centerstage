package org.firstinspires.ftc.teamcode.opmode.auton.blue;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Blue Left", group = "_Auto")
@Config
public class LeftBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {

        }
    }
}
