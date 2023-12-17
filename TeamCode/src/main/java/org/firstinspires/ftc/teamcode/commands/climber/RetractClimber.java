package org.firstinspires.ftc.teamcode.commands.climber;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Climber;

public class RetractClimber extends CommandBase {
    private final Climber climber;

    public RetractClimber(final Climber climber) {
        this.climber = climber;

        super.addRequirements(this.climber);
    }

    @Override
    public void initialize() {
        this.climber.setTarget(0);
    }
}