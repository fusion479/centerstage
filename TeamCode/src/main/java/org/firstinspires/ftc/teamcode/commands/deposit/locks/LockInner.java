package org.firstinspires.ftc.teamcode.commands.deposit;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Deposit;

public class LockInner extends CommandBase {
    private final Deposit deposit;

    public LockInner (final Deposit deposit) {
        this.deposit = deposit;

        super.addRequirements(this.deposit);
    }

    @Override
    public void initialize() {
        this.deposit.setInnerPosition(Deposit.LOCKINNER);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}