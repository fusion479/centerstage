package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeSetPower extends CommandBase {
    private final Intake intake;
    private final double power;

    public IntakeSetPower(final Intake intake, final double power) {
        this.intake = intake;
        this.power = power;

        super.addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        this.intake.setPower(this.power);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
