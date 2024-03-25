package org.firstinspires.ftc.teamcode.commands.lift;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class LiftDownALittle extends CommandBase {
    private final Lift lift;

    public LiftDownALittle(final Lift lift) {
        this.lift = lift;
        super.addRequirements(this.lift);

    }

    @Override
    public void initialize() {
        this.lift.setTarget(lift.getTarget() - Lift.INCREMENT);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
