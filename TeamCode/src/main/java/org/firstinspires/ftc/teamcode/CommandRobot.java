package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.arm.ArmAccepting;
import org.firstinspires.ftc.teamcode.commands.arm.ArmReady;
import org.firstinspires.ftc.teamcode.commands.arm.ArmScore;
import org.firstinspires.ftc.teamcode.commands.deposit.DepositAccepting;
import org.firstinspires.ftc.teamcode.commands.deposit.DepositReady;
import org.firstinspires.ftc.teamcode.commands.deposit.DepositScore;
import org.firstinspires.ftc.teamcode.commands.deposit.locks.LockInner;
import org.firstinspires.ftc.teamcode.commands.deposit.locks.LockOuter;
import org.firstinspires.ftc.teamcode.commands.deposit.locks.OpenInner;
import org.firstinspires.ftc.teamcode.commands.deposit.locks.OpenOuter;
import org.firstinspires.ftc.teamcode.commands.drivetrain.ManualDrive;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeAccepting;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeReady;
import org.firstinspires.ftc.teamcode.commands.launcher.Idle;
import org.firstinspires.ftc.teamcode.commands.launcher.Launch;
import org.firstinspires.ftc.teamcode.commands.lift.BottomLift;
import org.firstinspires.ftc.teamcode.commands.lift.HighLift;
import org.firstinspires.ftc.teamcode.commands.lift.LiftLower;
import org.firstinspires.ftc.teamcode.commands.lift.LiftRaise;
import org.firstinspires.ftc.teamcode.commands.lift.LowLift;
import org.firstinspires.ftc.teamcode.commands.lift.MediumLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GamepadTrigger;

public class CommandRobot extends Robot {
    private final Arm arm;
    private final Drivetrain drive;
    private final Lift lift;
    private final Launcher launcher;
    private final Intake intake;
    private final GamepadEx gamepad1;
    private final GamepadEx gamepad2;
    final Deposit deposit;
    private final GamepadTrigger intakeAccept;
    private final GamepadTrigger intakeReject;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean autoLocked = false;
    public final Commands COMMANDS = new Commands();

    public CommandRobot(final HardwareMap hwMap, final GamepadEx gamepad1, final GamepadEx gamepad2, final MultipleTelemetry telemetry) { // Create different bots for teleop, testing, and auton?
        this.deposit = new Deposit(hwMap, telemetry);
        this.arm = new Arm(hwMap, telemetry);
        this.drive = new Drivetrain(hwMap, telemetry);
        this.lift = new Lift(hwMap, telemetry);
        this.launcher = new Launcher(hwMap, telemetry);
        this.intake = new Intake(hwMap, telemetry);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        this.intakeAccept = new GamepadTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER, this.intake::setPower, this.gamepad1);
        this.intakeReject = new GamepadTrigger(GamepadKeys.Trigger.LEFT_TRIGGER, d -> this.intake.setPower(-d), this.gamepad1);

        this.drive.setDefaultCommand(new ManualDrive(this.drive, this.gamepad1));

        this.configureCommands();
    }

    public void configureCommands() {
        this.gamepad1.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(COMMANDS.ACCEPTING, COMMANDS.READY);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(COMMANDS.SCORE_LOW);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(COMMANDS.SCORE_HIGH);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(COMMANDS.SCORE_MID);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(COMMANDS.LIFT_RAISE);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(COMMANDS.LIFT_LOWER);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(COMMANDS.SCORE_ONE);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(COMMANDS.SCORE_TWO);
        this.gamepad2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(COMMANDS.LAUNCH);
        this.gamepad2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(COMMANDS.IDLE);
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

    public class Commands {
        public final Command ACCEPTING = new SequentialCommandGroup(
                new LowLift(lift),
                new ArmAccepting(arm),
                new IntakeAccepting(intake),
                new OpenInner(deposit),
                new OpenOuter(deposit),
                new DepositAccepting(deposit),
                new BottomLift(lift));

        public final Command READY = new SequentialCommandGroup(
                        new LockInner(deposit),
                        new LockOuter(deposit),
                        new LowLift(lift),
                        new ArmReady(arm),
                        new DepositReady(deposit),
                        new IntakeReady(intake),
                        new BottomLift(lift));

        public final Command SCORE_LOW = new ParallelCommandGroup(
                new LockOuter(deposit),
                new LockInner(deposit),
                new LowLift(lift),
                new ArmScore(arm),
                new DepositScore(deposit),
                new IntakeReady(intake));

        public final Command SCORE_HIGH = new ParallelCommandGroup(
                        new LockOuter(deposit),
                        new LockInner(deposit),
                        new HighLift(lift),
                        new ArmScore(arm),
                        new DepositScore(deposit),
                        new IntakeReady(intake));

        public final Command SCORE_MID = new ParallelCommandGroup(
                new LockOuter(deposit),
                new LockInner(deposit),
                new MediumLift(lift),
                new ArmScore(arm),
                new DepositScore(deposit),
                new IntakeReady(intake));

        public final Command LIFT_RAISE = new SequentialCommandGroup(
                new LockOuter(deposit),
                new LockInner(deposit),
                new ArmScore(arm),
                new DepositScore(deposit),
                new IntakeReady(intake),
                new LiftRaise(lift));

        public final Command LIFT_LOWER = new SequentialCommandGroup(
                new LockOuter(deposit),
                new LockInner(deposit),
                new ArmScore(arm),
                new DepositScore(deposit),
                new IntakeReady(intake),
                new LiftLower(lift));

        public final Command SCORE_ONE = new SequentialCommandGroup(
                new OpenOuter(deposit),
                new WaitCommand(175),
                new LiftRaise(lift),
                new InstantCommand(() -> {
                    autoLocked = false;
                    timer.reset();
                })
        );

        public final Command SCORE_TWO = new SequentialCommandGroup(
                new OpenInner(deposit),
                new InstantCommand(() -> autoLocked = false),
                new WaitCommand(175),
                new LiftRaise(lift),
                new InstantCommand(() -> {
                    autoLocked = false;
                    timer.reset();
                }));

        public final Command LAUNCH = new Launch(launcher);

        public final Command IDLE = new Idle(launcher);
    }
}
