package org.firstinspires.ftc.teamcode.commands.intake;


import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeAccepting extends CommandBase {
    private final Intake intake;

    public IntakeAccepting(final Intake intake) {
        this.intake = intake;

        super.addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        this.intake.setPosition(Intake.ACCEPTING_POS);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
