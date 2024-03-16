package org.firstinspires.ftc.teamcode.commands.launcher;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Launcher;

public class Idle extends CommandBase {
    private final Launcher launcher;

    public Idle(Launcher launcher) {
        this.launcher = launcher;

        super.addRequirements(this.launcher);
    }

    @Override
    public void initialize() {
        this.launcher.idle();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
