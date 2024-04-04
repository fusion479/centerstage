package org.firstinspires.ftc.teamcode.commands.lift;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class LiftRaise extends CommandBase {
    private final Lift lift;
    private final MultipleTelemetry multipleTelemetry;

    public LiftRaise(final Lift lift, MultipleTelemetry telemetry) {
        this.lift = lift;
        super.addRequirements(this.lift);
        this.multipleTelemetry = telemetry;

    }

    @Override
    public void initialize() {
        multipleTelemetry.addData("previous target: ", this.lift.getTarget());
        this.lift.setTarget(lift.getTarget() + Lift.INCREMENT);
        multipleTelemetry.addData("incremented target: ", this.lift.getTarget());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
