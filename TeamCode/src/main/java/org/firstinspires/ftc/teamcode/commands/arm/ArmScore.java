package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class ArmScore extends CommandBase {
    private final Arm arm;

    public ArmScore(Arm arm) {
        this.arm = arm;

        super.addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        this.arm.setPosition(Arm.SCORE_POS);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
