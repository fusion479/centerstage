package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class ArmDown extends CommandBase {
    private final Arm arm;

    public ArmDown(Arm arm) {
        this.arm = arm;

        super.addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        this.arm.setPosition(Arm.DOWN_POS);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
