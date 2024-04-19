package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.DemoDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class DemoDrive extends CommandBase {
    private final DemoDrivetrain drivetrain;
    private final GamepadEx gamepad;

    public DemoDrive(final DemoDrivetrain drivetrain, final GamepadEx gamepad) {
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
