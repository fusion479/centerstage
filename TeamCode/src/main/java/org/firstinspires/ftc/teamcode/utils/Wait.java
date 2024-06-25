package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Wait extends CommandBase{
    private ElapsedTime timer;
    private long duration;

    public Wait(long duration) {
        this.timer = new ElapsedTime();
        this.duration = duration;
    }

    @Override
    public void initialize() {
        this.timer.reset();
    }

    @Override
    public void execute() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public boolean isFinished() {
        return this.duration <= this.timer.milliseconds();
    }
}
