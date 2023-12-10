package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandRobot;

@TeleOp(name = "Main", group = "TeleOp")
public class Main extends CommandOpMode {
    private CommandRobot robot;

    public void initialize() {
        this.robot = new CommandRobot(this.hardwareMap);
        this.robot.enable();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        super.waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            robot.run(); // runs the command scheduler
        }

        robot.reset();
        robot.disable();
    }
}
