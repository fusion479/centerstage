package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.arm.ArmDown;
import org.firstinspires.ftc.teamcode.commands.arm.ArmUp;
import org.firstinspires.ftc.teamcode.commands.climber.ExtendClimber;
import org.firstinspires.ftc.teamcode.commands.climber.RetractClimber;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Climber;

public class CommandRobot extends Robot {
    private GamepadEx gamepad;

    private Climber climber;
    private Arm arm;

    public CommandRobot(final HardwareMap hwMap) {
        this.climber = new Climber(hwMap);
        this.arm = new Arm(hwMap);

        this.gamepad = new GamepadEx(new Gamepad()); // connect to right gamepad

        this.configureCommands();
        CommandScheduler.getInstance().run();
    }

    public void configureCommands() {
        // CLIMBER COMMANDS
        this.gamepad.getGamepadButton(GamepadKeys.Button.X).toggleWhenPressed(new ExtendClimber(this.climber), new RetractClimber(this.climber));

        // ARM COMMANDS
        this.gamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ArmUp(this.arm));
        this.gamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new ArmDown(this.arm));
    }
}
