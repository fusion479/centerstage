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
    private final ElapsedTime timer;
    public Commands commands;
    private GamepadTrigger intakeAccept;
    private GamepadTrigger intakeReject;
    private boolean autoLocked;

    public CommandRobot(final HardwareMap hwMap, final GamepadEx gamepad1, final GamepadEx gamepad2, final MultipleTelemetry telemetry) {
        this.timer = new ElapsedTime();
        this.autoLocked = false;

        this.deposit = new Deposit(hwMap, telemetry);
        this.drive = new Drivetrain(hwMap, telemetry);
        this.intake = new Intake(hwMap, telemetry);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        this.commands = new Commands(
                new Arm(hwMap, telemetry),
                new Lift(hwMap, telemetry),
                new Launcher(hwMap, telemetry),
                new Intake(hwMap, telemetry),
                this.deposit,
                this.timer,
                () -> this.autoLocked = false,
                this.timer::reset);
    }

    public void configureControls() {
        this.gamepad1.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(this.commands.intake, this.commands.ready);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(this.commands.scoreLow);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(this.commands.scoreHigh);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(this.commands.scoreMid);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(this.commands.liftRaise);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(this.commands.liftLower);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(this.commands.scoreOne);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(this.commands.scoreTwo);
        this.gamepad2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(this.commands.launch);
        this.gamepad2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(this.commands.launchIdle);

        this.intakeAccept = new GamepadTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER, this.intake::setPower, this.gamepad1);
        this.intakeReject = new GamepadTrigger(GamepadKeys.Trigger.LEFT_TRIGGER, d -> this.intake.setPower(-d), this.gamepad1);

        this.drive.setDefaultCommand(new ManualDrive(this.drive, this.gamepad1));
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
