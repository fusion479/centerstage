package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class ManualDrive extends CommandBase {
    private final Drivetrain drivetrain;
    private final GamepadEx gamepad;

    public ManualDrive(final Drivetrain drivetrain, final GamepadEx gamepad) {
        this.drivetrain = drivetrain;
        this.gamepad = gamepad;

        super.addRequirements(this.drivetrain);
    }

    @Override
    public void execute() {
        this.drivetrain.manualDrive(this.gamepad);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
