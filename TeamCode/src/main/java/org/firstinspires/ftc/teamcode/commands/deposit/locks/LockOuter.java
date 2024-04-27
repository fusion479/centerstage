package org.firstinspires.ftc.teamcode.commands.deposit.locks;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Deposit;

public class LockOuter extends CommandBase {
    private final Deposit deposit;

    public LockOuter(final Deposit deposit) {
        this.deposit = deposit;
    }

    @Override
    public void initialize() {
        this.deposit.setOuterPosition(Deposit.LOCK_OUTER);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
