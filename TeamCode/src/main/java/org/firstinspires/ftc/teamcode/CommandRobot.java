package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.Commands;
import org.firstinspires.ftc.teamcode.commands.deposit.locks.LockInner;
import org.firstinspires.ftc.teamcode.commands.deposit.locks.LockOuter;
import org.firstinspires.ftc.teamcode.commands.drivetrain.ManualDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GamepadTrigger;

public class CommandRobot extends Robot {
    private final Drivetrain drive;
    private final Deposit deposit;
    private final Intake intake;
    private final GamepadEx gamepad1;
    private final GamepadEx gamepad2;
    private final GamepadTrigger intakeAccept;
    private final GamepadTrigger intakeReject;
    private final ElapsedTime timer = new ElapsedTime();
    public Commands commands;
    private boolean autoLocked = false;

    public CommandRobot(final HardwareMap hwMap, final GamepadEx gamepad1, final GamepadEx gamepad2, final MultipleTelemetry telemetry) {
        this.deposit = new Deposit(hwMap, telemetry);
        this.drive = new Drivetrain(hwMap, telemetry);
        this.intake = new Intake(hwMap, telemetry);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        this.intakeAccept = new GamepadTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER, this.intake::setPower, this.gamepad1);
        this.intakeReject = new GamepadTrigger(GamepadKeys.Trigger.LEFT_TRIGGER, d -> this.intake.setPower(-d), this.gamepad1);
        this.commands = new Commands(new Arm(hwMap, telemetry), new Lift(hwMap, telemetry), new Launcher(hwMap, telemetry), new Intake(hwMap, telemetry), this.deposit, this.timer, () -> this.autoLocked = false, this.timer::reset);

        this.drive.setDefaultCommand(new ManualDrive(this.drive, this.gamepad1));

        this.configureCommands();
    }

    public void configureCommands() {
        this.gamepad1.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(this.commands.intake, this.commands.ready);

        // LIFT LOW
        this.gamepad1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(this.commands.scoreLow);
        // LIFT HIGH
        this.gamepad1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(this.commands.scoreHigh);

        // LIFT MEDIUM
        this.gamepad1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(this.commands.scoreMid);

        // LIFT UP A LITTLE
        this.gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(this.commands.liftRaise);

        // LIFT DOWN A LITTLE
        this.gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(this.commands.liftLower);

        // SCORE PIXEL ONE
        this.gamepad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(this.commands.scoreOne);

        // SCORE PIXEL TWO
        this.gamepad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(this.commands.scoreTwo);

        // LAUNCHER COMMANDS
        this.gamepad2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(this.commands.launch);
        this.gamepad2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(this.commands.launchIdle);
    }

    public void updateTriggers() {
        this.intakeAccept.update();
        this.intakeReject.update();
    }

    public void senseColor() {
        if (this.deposit.hasPixel() && !this.autoLocked && timer.milliseconds() >= 300) {
            new LockInner(this.deposit).schedule();
            new LockOuter(this.deposit).schedule();
            this.autoLocked = true;
        }
    }
}
