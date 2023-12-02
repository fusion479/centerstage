package org.firstinspires.ftc.teamcode.commands.climber;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Climber;

public class RetractClimber extends CommandBase {
    private final Climber climber;

    public RetractClimber(final Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        this.climber.setTarget(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}