package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class ArmReady extends CommandBase {
    private final Arm arm;

    public ArmReady(Arm arm) {
        this.arm = arm;

        super.addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        this.arm.setPosition(Arm.READY_POS);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
