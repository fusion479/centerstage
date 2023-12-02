package org.firstinspires.ftc.teamcode.commands.climber;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Climber;

public class ExtendClimber extends CommandBase {
    private final Climber climber;

    public ExtendClimber(final Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        this.climber.setTarget(50); // or other value
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
