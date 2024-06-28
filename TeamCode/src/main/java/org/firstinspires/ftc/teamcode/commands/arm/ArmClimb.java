package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class ArmClimb extends CommandBase {
    private final Arm arm;

    public ArmClimb(Arm arm) {
        this.arm = arm;

        super.addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        this.arm.setPosition(Arm.CLIMB_POS);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
