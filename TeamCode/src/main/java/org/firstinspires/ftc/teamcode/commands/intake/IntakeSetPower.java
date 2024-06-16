package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeSetPower extends CommandBase {
    private final Intake intake;
    private final double power;
    private final int duration;
    private final ElapsedTime timer;

    public IntakeSetPower(final Intake intake, final double power, final int duration) {
        this.intake = intake;
        this.power = power;
        this.duration = duration;
        this.timer = new ElapsedTime();

        super.addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        this.timer.reset();
    }

    @Override
    public void execute() {
        this.intake.setPower(this.power);
    }

    @Override
    public boolean isFinished() {
        if (this.timer.milliseconds() >= this.duration) {
            this.intake.setPower(0);
            return true;
        }
        return false;
    }
}
