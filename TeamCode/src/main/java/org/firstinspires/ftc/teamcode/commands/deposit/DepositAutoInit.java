package org.firstinspires.ftc.teamcode.commands.deposit;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Deposit;

public class DepositAutoInit extends CommandBase {
    private final Deposit deposit;

    public DepositAutoInit(final Deposit deposit) {
        this.deposit = deposit;

        super.addRequirements(this.deposit);
    }

    @Override
    public void initialize() {
        this.deposit.setPosition(Deposit.AUTO_INIT_POS);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
