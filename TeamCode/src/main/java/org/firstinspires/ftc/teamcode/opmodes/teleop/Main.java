package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;

public class Main extends CommandOpMode {

    public void initialize() {

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
