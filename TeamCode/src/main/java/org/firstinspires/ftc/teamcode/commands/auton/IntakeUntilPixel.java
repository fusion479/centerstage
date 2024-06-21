package org.firstinspires.ftc.teamcode.commands.auton;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.deposit.locks.LockInner;
import org.firstinspires.ftc.teamcode.commands.deposit.locks.LockOuter;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeUntilPixel extends CommandBase {
    private final Intake intake;
    private final Deposit deposit;
    private final ElapsedTime timer;

    public IntakeUntilPixel(Deposit deposit, Intake intake) {
        this.deposit = deposit;
        this.intake = intake;
        this.timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        this.intake.setPower(-1);
    }

    @Override
    public void execute() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public boolean isFinished() {
        if (!this.deposit.hasOuterPixel() || !this.deposit.hasOuterPixel()) {
            this.timer.reset();
        }

        if (this.deposit.hasOuterPixel() && this.deposit.hasInnerPixel() && timer.milliseconds() >= 750) {
            new LockInner(this.deposit).schedule();
            new LockOuter(this.deposit).schedule();

            this.intake.setPower(0);
            CommandScheduler.getInstance().run();
            return true;
        }

        return false;
    }
}
