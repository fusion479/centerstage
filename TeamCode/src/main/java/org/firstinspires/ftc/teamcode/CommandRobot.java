package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.common.subtyping.qual.Bottom;
import org.firstinspires.ftc.teamcode.commands.arm.ArmDown;
import org.firstinspires.ftc.teamcode.commands.arm.ArmReady;
import org.firstinspires.ftc.teamcode.commands.arm.ArmUp;
import org.firstinspires.ftc.teamcode.commands.deposit.DepositAccepting;
import org.firstinspires.ftc.teamcode.commands.deposit.DepositReady;
import org.firstinspires.ftc.teamcode.commands.deposit.DepositScore;
import org.firstinspires.ftc.teamcode.commands.deposit.OpenInner;
import org.firstinspires.ftc.teamcode.commands.deposit.locks.OpenOuter;
import org.firstinspires.ftc.teamcode.commands.drivetrain.ManualDrive;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeDown;
import org.firstinspires.ftc.teamcode.commands.launcher.Idle;
import org.firstinspires.ftc.teamcode.commands.launcher.Launch;
import org.firstinspires.ftc.teamcode.commands.lift.BottomLift;
import org.firstinspires.ftc.teamcode.commands.lift.HighLift;
import org.firstinspires.ftc.teamcode.commands.lift.LowLift;
import org.firstinspires.ftc.teamcode.commands.lift.MediumLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class CommandRobot extends Robot {
    private final Arm arm;
    private final Drivetrain drive;
    private final Lift lift;
    private final Launcher launcher;
    private final Intake intake;
    private final GamepadEx gamepad1;
    private final GamepadEx gamepad2;
    private final Deposit deposit;
    public boolean ready;

    public CommandRobot(final HardwareMap hwMap, final GamepadEx gamepad1, final GamepadEx gamepad2, final MultipleTelemetry telemetry) { // Create different bots for teleop, testing, and auton?
        this.arm = new Arm(hwMap, telemetry);
        this.drive = new Drivetrain(hwMap, telemetry);
        this.lift = new Lift(hwMap, telemetry);
        this.launcher = new Launcher(hwMap, telemetry);
        this.intake = new Intake(hwMap, telemetry);
        this.deposit = new Deposit(hwMap, telemetry);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        ready = false;

        this.drive.setDefaultCommand(new ManualDrive(this.drive, this.gamepad1));

        this.configureCommands();
    }
    public void configureCommands() {
        // Ready - figure out how to togle
        this.gamepad1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new SequentialCommandGroup(
                        new BottomLift(this.lift),
                        new IntakeDown(this.intake),
                        new ArmReady(this.arm),
                        new DepositReady(this.deposit),
                        new OpenInner(this.deposit),
                        new OpenOuter(this.deposit),
                        new DepositAccepting(this.deposit)
                        ));



        // LIFT LOW
        this.gamepad1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new SequentialCommandGroup(
                        new LowLift(this.lift),
                        new ArmUp(this.arm),
                        new DepositScore(this.deposit)
                ));
        // LIFT HIGH
        this.gamepad1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new SequentialCommandGroup(
                        new HighLift(this.lift),
                        new ArmUp(this.arm),
                        new DepositScore(this.deposit)
                ));
        // LIFT MEDIUM
        this.gamepad1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new SequentialCommandGroup(
                        new MediumLift(this.lift),
                        new ArmUp(this.arm),
                        new DepositScore(this.deposit)
                ));
        // SCORE PIXEL ONE
        this.gamepad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new OpenOuter(this.deposit));
        // SCORE PIXEL TWO
        this.gamepad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new OpenInner(this.deposit));

        // LAUNCHER COMMANDS
        this.gamepad2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new Launch(this.launcher));
        this.gamepad2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new Idle(this.launcher));
        //  CLIMB


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
