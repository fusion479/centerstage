package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import java.util.function.DoubleConsumer;

public class GamepadTrigger {
    private final GamepadKeys.Trigger trigger;
    private final DoubleConsumer command;
    private final GamepadEx gamepad;
    private final MultipleTelemetry multipleTelemetry;
    private boolean isReleased = false;

    public GamepadTrigger(GamepadKeys.Trigger trigger, DoubleConsumer command, GamepadEx gamepad) {
        this.trigger = trigger;
        this.command = command;
        this.gamepad = gamepad;
        this.multipleTelemetry = new MultipleTelemetry();
    }

    public GamepadTrigger(GamepadKeys.Trigger trigger, DoubleConsumer command, GamepadEx gamepad, MultipleTelemetry multipleTelemetry) {
        this.trigger = trigger;
        this.command = command;
        this.gamepad = gamepad;
        this.multipleTelemetry = multipleTelemetry;
    }

    public void update() {
        if (this.gamepad.getTrigger(this.trigger) > 0) {
            this.isReleased = false;
            this.multipleTelemetry.addData("Trigger Value: ", this.gamepad.getTrigger(this.trigger));
            command.accept(this.gamepad.getTrigger(this.trigger));
        } else if (!isReleased) {
            command.accept(0);
        } else {
            this.isReleased = true;
        }
    }
}
