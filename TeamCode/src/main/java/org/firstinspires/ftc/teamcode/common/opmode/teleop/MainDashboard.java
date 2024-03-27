package org.firstinspires.ftc.teamcode.common.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.subsystem.Robot;
import org.firstinspires.ftc.teamcode.common.subsystem.RobotDashboard;

@TeleOp(name = "DASHBOARD ABCDEFGHIJKLMNOPQRSTUVWXYZ", group = "!!!")
public class MainDashboard extends LinearOpMode {
    RobotDashboard robot = new RobotDashboard();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            robot.update(gamepad1, gamepad2);

        }
    }
}
