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
    private final ElapsedTime pixelTimer;
    private final ElapsedTime durationTimer;

    public IntakeUntilPixel(Deposit deposit, Intake intake) {
        this.deposit = deposit;
        this.intake = intake;
        this.pixelTimer = new ElapsedTime();
        this.durationTimer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        this.intake.setPower(-1);
        this.pixelTimer.reset();
        this.durationTimer.reset();
    }

    @Override
    public void execute() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public boolean isFinished() {
        if (this.durationTimer.milliseconds() >= 5000) {
            return true;
        }

        if (!this.deposit.hasOuterPixel() || !this.deposit.hasOuterPixel()) {
            this.pixelTimer.reset();
        }

        if (this.deposit.hasOuterPixel() && this.deposit.hasInnerPixel() && pixelTimer.milliseconds() >= 825) {
            new LockInner(this.deposit).schedule();
            new LockOuter(this.deposit).schedule();

            this.intake.setPower(0);
            CommandScheduler.getInstance().run();
            return true;
        }

        return false;
    }
}
