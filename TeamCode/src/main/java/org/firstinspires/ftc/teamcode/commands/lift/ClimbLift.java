package org.firstinspires.ftc.teamcode.commands.lift;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class ClimbLift extends CommandBase {
    private final Lift lift;

    public ClimbLift(final Lift lift) {
        this.lift = lift;

        super.addRequirements(this.lift);
    }

    @Override
    public void initialize() {
        this.lift.setTarget(Lift.CLIMB_POS);
    }

    @Override
    public boolean isFinished() {
        return this.lift.isFinished();
    }
}