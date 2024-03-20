package org.firstinspires.ftc.teamcode.commands.lift;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class MediumLift extends CommandBase {
    private final Lift lift;

    public MediumLift(final Lift lift) {
        this.lift = lift;

        super.addRequirements(this.lift);
    }


    @Override
    public void initialize() {
        this.lift.setTarget(1000);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
