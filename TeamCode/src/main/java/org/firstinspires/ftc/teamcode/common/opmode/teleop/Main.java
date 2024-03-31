package org.firstinspires.ftc.teamcode.common.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.subsystem.Robot;

@TeleOp(name = "_DANIEL", group = "!!!")
public class Main extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            robot.update(gamepad1, gamepad2);

        }
    }
}
