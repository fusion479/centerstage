package org.firstinspires.ftc.teamcode.commands.lift;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.CommandRobot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.AutonCommandBase;

public class LowLift extends CommandBase {
    private final Lift lift;

    public LowLift(final Lift lift) {
        this.lift = lift;

        super.addRequirements(this.lift);
    }

    @Override
    public void initialize() {
        this.lift.setTarget(Lift.LOW_POS);
    }

    @Override
    public boolean isFinished() {
        return this.lift.isFinished();
    }
}
