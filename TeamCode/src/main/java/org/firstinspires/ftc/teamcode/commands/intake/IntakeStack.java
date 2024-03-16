package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeStack extends CommandBase {
    private final Intake intake;

    public IntakeStack(final Intake intake) {
        this.intake = intake;

        super.addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        this.intake.setPosition(Intake.STACK_POS);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
