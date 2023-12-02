package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Robot;

public class Main extends CommandOpMode {
    private Robot robot;

    public void initialize() {
        this.robot = new Robot(this.hardwareMap);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        super.waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            CommandScheduler.getInstance().run();
        }

        CommandScheduler.getInstance().reset();
    }
}
