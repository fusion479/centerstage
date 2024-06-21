package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.CommandRobot;

public class AutonCommandBase extends CommandBase {
    private final CommandRobot.Type type;

    public AutonCommandBase(final CommandRobot.Type type) {
        this.type = type;
    }

    public void runScheduler() {
        if (this.type == CommandRobot.Type.AUTON) {
            CommandScheduler.getInstance().run();
        }
    }

    @Override
    public void execute() {
        this.runScheduler();
    }
}