package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
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
import org.firstinspires.ftc.teamcode.commands.intake.IntakeStack;
import org.firstinspires.ftc.teamcode.commands.launcher.Idle;
import org.firstinspires.ftc.teamcode.commands.launcher.Launch;
import org.firstinspires.ftc.teamcode.commands.lift.BottomLift;
import org.firstinspires.ftc.teamcode.commands.lift.HighLift;
import org.firstinspires.ftc.teamcode.commands.lift.LiftLower;
import org.firstinspires.ftc.teamcode.commands.lift.LiftRaise;
import org.firstinspires.ftc.teamcode.commands.lift.LowLift;
import org.firstinspires.ftc.teamcode.commands.lift.MediumLift;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GamepadTrigger;

public class CommandRobot extends Robot {
    // SUBSYSTEMS & CONTROLLERS
    private final Arm arm;
    private final Drivetrain drive;
    private final Lift lift;
    private final Launcher launcher;
    public final Intake intake;
    private final GamepadEx gamepad1;
    private final GamepadEx gamepad2;
    private final Deposit deposit;

    // COMMANDS
    public final Command accepting;
    public final Command ready;
    public final Command scoreLow;
    public final Command scoreHigh;
    public final Command scoreMid;
    public final Command liftRaise;
    public final Command liftLower;
    public final Command scoreOne;
    public final Command scoreTwo;

    public final Command stack;
    public final Command launch;
    public final Command idle;

    private final GamepadTrigger intakeAccept;
    private final GamepadTrigger intakeReject;

    // MISC. VARIABLES
    private final ElapsedTime timer;
    private boolean locked;

    public CommandRobot(final HardwareMap hwMap, final GamepadEx gamepad1, final GamepadEx gamepad2, final MultipleTelemetry telemetry, Pose2d startPose) { // Create different bots for teleop, testing, and auton?
        this.timer = new ElapsedTime();
        this.locked = false;

        this.deposit = new Deposit(hwMap, telemetry);
        this.arm = new Arm(hwMap, telemetry);
        this.drive = new Drivetrain(hwMap, telemetry, startPose);
        this.lift = new Lift(hwMap, telemetry);
        this.launcher = new Launcher(hwMap, telemetry);
        this.intake = new Intake(hwMap, telemetry);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        this.accepting = new SequentialCommandGroup(
                new LowLift(this.lift),
                new DepositAccepting(this.deposit),
                new WaitCommand(500),
                new ArmAccepting(this.arm),
                new IntakeAccepting(this.intake),
                new OpenInner(this.deposit),
                new OpenOuter(this.deposit),
                new BottomLift(this.lift));

        this.ready = new SequentialCommandGroup(
                new LockInner(this.deposit),
                new LockOuter(this.deposit),
                new LowLift(this.lift),
                new ArmReady(this.arm),
                new DepositReady(this.deposit),
                new IntakeReady(this.intake),
                new BottomLift(this.lift));

        this.scoreLow = new ParallelCommandGroup(
                new LockOuter(this.deposit),
                new LockInner(this.deposit),
                new LowLift(this.lift),
                new ArmScore(this.arm),
                new DepositScore(this.deposit),
                new IntakeReady(this.intake));

        this.scoreHigh = new ParallelCommandGroup(
                new LockOuter(this.deposit),
                new LockInner(this.deposit),
                new HighLift(this.lift),
                new ArmScore(this.arm),
                new DepositScore(this.deposit),
                new IntakeReady(this.intake));

        this.scoreMid = new ParallelCommandGroup(
                new LockOuter(this.deposit),
                new LockInner(this.deposit),
                new MediumLift(this.lift),
                new ArmScore(this.arm),
                new DepositScore(this.deposit),
                new IntakeReady(this.intake));

        this.liftRaise = new SequentialCommandGroup(
                new LockOuter(this.deposit),
                new LockInner(this.deposit),
                new ArmScore(this.arm),
                new DepositScore(this.deposit),
                new IntakeReady(this.intake),
                new LiftRaise(this.lift));

        this.liftLower = new SequentialCommandGroup(
                new LockOuter(this.deposit),
                new LockInner(this.deposit),
                new ArmScore(this.arm),
                new DepositScore(this.deposit),
                new IntakeReady(this.intake),
                new LiftLower(this.lift));

        this.scoreOne = new SequentialCommandGroup(
                new OpenOuter(this.deposit),
                new WaitCommand(175),
                new LiftRaise(this.lift),
                new InstantCommand(() -> {
                    this.locked = false;
                    this.timer.reset();
                }));

        this.scoreTwo = new SequentialCommandGroup(
                new OpenInner(this.deposit),
                new InstantCommand(() -> locked = false),
                new WaitCommand(175),
                new LiftRaise(this.lift),
                new InstantCommand(() -> {
                    this.locked = false;
                    this.timer.reset();
                }));

        this.stack = new SequentialCommandGroup(
                new LowLift(this.lift),
                new DepositAccepting(this.deposit),
                new WaitCommand(500),
                new ArmAccepting(this.arm),
                new IntakeStack(this.intake),
                new OpenInner(this.deposit),
                new OpenOuter(this.deposit),
                new BottomLift(this.lift)
                );

        this.idle = new Idle(this.launcher);
        this.launch = new Launch(this.launcher);

        this.intakeAccept = new GamepadTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER, this.intake::setPower, this.gamepad1);
        this.intakeReject = new GamepadTrigger(GamepadKeys.Trigger.LEFT_TRIGGER, d -> this.intake.setPower(-d), this.gamepad1);

        this.drive.setDefaultCommand(new ManualDrive(this.drive, this.gamepad1));
    }

    public void configureCommands() {
        this.gamepad1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(this.accepting);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(this.scoreLow);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(this.scoreHigh);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(this.scoreMid);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(this.liftRaise);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(this.liftLower);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(this.scoreOne);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new SequentialCommandGroup(
                        this.scoreTwo,
                        new WaitCommand(500),
                        this.accepting
                ));
        this.gamepad2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(this.launch);
        this.gamepad2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(this.idle);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new ParallelCommandGroup(
                        new LockOuter(this.deposit),
                        new LockInner(this.deposit)
                ));
    }

    public void updateTriggers() {
        this.intakeAccept.update();
        this.intakeReject.update();
    }

    public void senseColor() {
    }

    public MecanumDrive getDrive() {
        return this.drive.drive;
    }
}
