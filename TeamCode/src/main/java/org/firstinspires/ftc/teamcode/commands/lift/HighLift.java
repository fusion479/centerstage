package org.firstinspires.ftc.teamcode.commands.lift;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class HighLift extends CommandBase {
    private final Lift lift;

    public HighLift(final Lift lift) {
        this.lift = lift;

        super.addRequirements(this.lift);
    }

    @Override
    public void initialize() {
        this.lift.setTarget(Lift.HIGH_POS);
    }

    @Override
    public boolean isFinished() {
        return this.lift.isFinished();
    }
}
