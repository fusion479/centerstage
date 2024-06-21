package org.firstinspires.ftc.teamcode.commands.auton;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeSetPower extends CommandBase {
    private final Intake intake;
    private final int duration;
    private final ElapsedTime timer;

    public IntakeSetPower(final Intake intake, final int duration) {
        this.intake = intake;
        this.duration = duration;
        this.timer = new ElapsedTime();

        super.addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        this.timer.reset();
        this.intake.setPower(1);
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
