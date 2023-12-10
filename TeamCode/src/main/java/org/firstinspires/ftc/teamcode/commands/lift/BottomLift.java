package org.firstinspires.ftc.teamcode.commands.lift;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class BottomLift extends CommandBase {
    private final Lift lift;

    public BottomLift(final Lift lift) {
        this.lift = lift;
    }

    @Override
    public void initialize() {
        this.lift.setTarget(0);
    }
}