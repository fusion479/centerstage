package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandGroupBase;

public class CommandGroupAction implements Action {
    private final CommandGroupBase command;
    private boolean initialized = false;
    private boolean finished = false;

    public CommandGroupAction(CommandGroupBase command) {
        this.command = command;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            command.initialize();
            initialized = true;
        } else if (command.isFinished()) {
            command.end(false);
            finished = true;
        } else {
            command.execute();
        }
        return !finished;
    }
}