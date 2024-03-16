package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import org.firstinspires.ftc.teamcode.commands.arm.ArmDown;
import org.firstinspires.ftc.teamcode.commands.arm.ArmUp;
import org.firstinspires.ftc.teamcode.commands.drivetrain.ManualDrive;
import org.firstinspires.ftc.teamcode.commands.launcher.Launch;
import org.firstinspires.ftc.teamcode.commands.lift.LowLift;
import org.firstinspires.ftc.teamcode.commands.lift.MediumLift;

public class CommandRobot extends Robot {
    private final Arm arm;
    private final Drivetrain drive;
    private final Lift lift;
    private final Launcher launcher;
    private final Intake intake;

    private final GamepadEx gamepad1;
    private final GamepadEx gamepad2;

    public CommandRobot(final HardwareMap hwMap, final GamepadEx gamepad1, final GamepadEx gamepad2, final MultipleTelemetry telemetry) { // Create different bots for teleop, testing, and auton?
        this.arm = new Arm(hwMap, telemetry);
        this.drive = new Drivetrain(hwMap, telemetry);
        this.lift = new Lift(hwMap, telemetry);
        this.launcher = new Launcher(hwMap, telemetry);
        this.intake = new Intake(hwMap, telemetry);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        this.drive.setDefaultCommand(new ManualDrive(this.drive, this.gamepad1));

        this.configureCommands();
    }

    public void configureCommands() {
        // ARM COMMANDS
        this.gamepad1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ArmUp(this.arm));
        this.gamepad1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ArmDown(this.arm));

        // LAUNCHER COMMANDS
        this.gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new Launch(this.launcher));

        // LIFT COMMANDS
        this.gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new MediumLift(this.lift));
        this.gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new LowLift(this.lift));
    }

    public void intakeTriggers() {
        double rTrigValue = this.gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        double lTrigValue = this.gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

        if (rTrigValue > 0.05) {
            this.intake.setPower(rTrigValue);
        } else {
            this.intake.setPower(lTrigValue);
        }
    }
}
