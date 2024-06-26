package org.firstinspires.ftc.teamcode.commands.auton;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeSetPower extends CommandBase {
    private final Intake intake;
    private final int duration;
    private final ElapsedTime timer;
    private final double power;

    public IntakeSetPower(final Intake intake, final int duration, final double power) {
        this.intake = intake;
        this.duration = duration;
        this.timer = new ElapsedTime();
        this.power = power;

        super.addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        this.timer.reset();
        this.intake.setPower(this.power);
    }

    @Override
    public void execute() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public boolean isFinished() {
        if (this.timer.milliseconds() >= this.duration) {
            this.intake.setPower(0);
            CommandScheduler.getInstance().run();
            return true;
        }
        return false;
    }
}
