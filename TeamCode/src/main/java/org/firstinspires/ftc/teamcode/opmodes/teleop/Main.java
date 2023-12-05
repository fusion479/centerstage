package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.CommandRobot;

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
            robot.run();
        }

        robot.reset();
        robot.disable();
    }
}
