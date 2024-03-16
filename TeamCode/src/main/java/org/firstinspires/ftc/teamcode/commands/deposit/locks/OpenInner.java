package org.firstinspires.ftc.teamcode.commands.deposit;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Deposit;

public class OpenInner extends CommandBase {
    private final Deposit deposit;

    public OpenInner(final Deposit deposit) {
        this.deposit = deposit;

        super.addRequirements(this.deposit);
    }

    @Override
    public void initialize() {
        this.deposit.setInnerPosition(Deposit.OPENINNER);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
