package org.firstinspires.ftc.teamcode.utils;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import java.util.function.DoubleConsumer;

public class GamepadTrigger {
    private GamepadKeys.Trigger trigger;
    private DoubleConsumer command;
    private TriggerReader triggerReader;
    private GamepadEx gamepad;

    public GamepadTrigger(GamepadKeys.Trigger trigger, DoubleConsumer command, GamepadEx gamepad) {
        this.trigger = trigger;
        this.command = command;
        this.gamepad = gamepad;
        this.triggerReader = new TriggerReader(gamepad, trigger);
    }

    public void update() {
        if (triggerReader.isDown()) {
            command.accept(this.gamepad.getTrigger(this.trigger));
        }
    }
}
