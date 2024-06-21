package org.firstinspires.ftc.teamcode.commands.lift;

import org.firstinspires.ftc.teamcode.CommandRobot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.AutonCommandBase;

public class LowLift extends AutonCommandBase {
    private final Lift lift;

    public LowLift(final Lift lift) {
        super(CommandRobot.Type.AUTON);
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
