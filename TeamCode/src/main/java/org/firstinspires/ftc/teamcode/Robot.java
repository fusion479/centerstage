package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.climber.ExtendClimber;
import org.firstinspires.ftc.teamcode.commands.climber.RetractClimber;
import org.firstinspires.ftc.teamcode.subsystems.Climber;

public class Robot {
    private Climber climber;
    private GamepadEx gamepad;

    public Robot(final HardwareMap hwMap) {
        this.climber = new Climber(hwMap);
        this.gamepad = new GamepadEx(new Gamepad()); // connect to right gamepad

        this.configureCommands();
    }

    public void configureCommands() {
        // CLIMBER COMMANDS
        this.gamepad.getGamepadButton(GamepadKeys.Button.X).toggleWhenPressed(new ExtendClimber(this.climber), new RetractClimber(this.climber));
    }
}
