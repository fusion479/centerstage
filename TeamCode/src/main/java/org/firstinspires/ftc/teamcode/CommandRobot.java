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
import org.firstinspires.ftc.teamcode.commands.arm.ArmClimb;
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
import org.firstinspires.ftc.teamcode.commands.lift.ClimbLift;
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
    // COMMANDS
    public final Command accepting, ready, scoreLow, scoreHigh, scoreMid, liftRaise, liftLower, scoreOne, scoreTwo, stack, launch, idle, scoreBottom, climbDown, climb;
    // SUBSYSTEMS & CONTROLLERS
    private final Arm arm;
    private final Drivetrain drive;
    private final Intake intake;
    private final Lift lift;
    private final Launcher launcher;
    private final GamepadEx gamepad1;
    private final GamepadEx gamepad2;
    private final Deposit deposit;
    private final Type type;
    // MISC. VARIABLES
    private final ElapsedTime timer;
    private GamepadTrigger intakeAccept;
    private GamepadTrigger intakeReject;
    private boolean locked;

    public CommandRobot(final HardwareMap hwMap, final GamepadEx gamepad1, final GamepadEx gamepad2, final MultipleTelemetry telemetry, final Pose2d startPose, final Type type) {
        this.timer = new ElapsedTime();
        this.locked = false;
        this.type = type;

        this.deposit = new Deposit(hwMap, telemetry);
        this.arm = new Arm(hwMap, telemetry);
        this.lift = new Lift(hwMap, telemetry);
        this.drive = new Drivetrain(hwMap, telemetry, startPose, lift);
        this.launcher = new Launcher(hwMap, telemetry);
        this.intake = new Intake(hwMap, telemetry);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        this.accepting = new SequentialCommandGroup(
                new LowLift(this.lift),
                new IntakeAccepting(this.intake),
                new ArmScore(this.arm),
                new WaitCommand(300),
                new DepositAccepting(this.deposit),
                new WaitCommand(300),
                new ArmAccepting(this.arm),
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
                new DepositScore(this.deposit));

        this.scoreHigh = new ParallelCommandGroup(
                new LockOuter(this.deposit),
                new LockInner(this.deposit),
                new HighLift(this.lift),
                new ArmScore(this.arm),
                new DepositScore(this.deposit));

        this.scoreMid = new ParallelCommandGroup(
                new LockOuter(this.deposit),
                new LockInner(this.deposit),
                new MediumLift(this.lift),
                new ArmScore(this.arm),
                new DepositScore(this.deposit));

        this.liftRaise = new SequentialCommandGroup(
                new LockOuter(this.deposit),
                new LockInner(this.deposit),
                new LiftRaise(this.lift));

        this.liftLower = new SequentialCommandGroup(
                new LockOuter(this.deposit),
                new LockInner(this.deposit),
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
                new WaitCommand(250),
                new LiftRaise(this.lift),
                new InstantCommand(() -> {
                    this.locked = false;
                    this.timer.reset();
                }),
                new WaitCommand(300),
                new LowLift(this.lift),
                new DepositAccepting(this.deposit),
                new WaitCommand(500),
                new ArmAccepting(this.arm),
                new IntakeAccepting(this.intake),
                new OpenInner(this.deposit),
                new OpenOuter(this.deposit),
                new BottomLift(this.lift));

        this.scoreBottom = new ParallelCommandGroup(
                new LockOuter(this.deposit),
                new LockInner(this.deposit),
                new BottomLift(this.lift),
                new ArmScore(this.arm),
                new DepositScore(this.deposit));

        this.stack = new SequentialCommandGroup(
                new LockInner(this.deposit), // yellow pixel in inner
                new LowLift(this.lift),
                new IntakeStack(this.intake),
                new ArmScore(this.arm),
                new WaitCommand(300),
                new DepositAccepting(this.deposit),
                new WaitCommand(300),
                new ArmAccepting(this.arm),
                new BottomLift(this.lift),
                new OpenOuter(this.deposit));

        this.climb = new SequentialCommandGroup(
                new ClimbLift(this.lift),
                new ArmClimb(this.arm),
                new WaitCommand(300),
                new DepositAccepting(this.deposit),
                new IntakeReady(this.intake)
        );

        this.climbDown = new SequentialCommandGroup(
                new IntakeReady(this.intake),
                new ArmClimb(this.arm),
                new InstantCommand(() -> this.intake.setPosition(Intake.INIT_POS)),
                new DepositAccepting(this.deposit),
                new BottomLift(this.lift)
        );

        this.idle = new Idle(this.launcher);
        this.launch = new Launch(this.launcher);

        if (this.type == Type.TELEOP) {
            this.intakeAccept = new GamepadTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER, d -> this.intake.setPower(-d), this.gamepad2);
            this.intakeReject = new GamepadTrigger(GamepadKeys.Trigger.LEFT_TRIGGER, this.intake::setPower, this.gamepad2);

            this.drive.setDefaultCommand(new ManualDrive(this.drive, this.gamepad2));
            this.configureCommands();
        }
    }

    public void configureCommands() {
        this.gamepad1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(this.accepting);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(this.scoreLow);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(this.scoreHigh);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(this.scoreMid);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(this.liftRaise);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(this.liftLower);
        this.gamepad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(this.scoreOne);
        this.gamepad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(this.scoreTwo);
        this.gamepad2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(this.launch);
        this.gamepad2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(this.idle);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(this.climb);
        this.gamepad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(this.climbDown);
    }

    public void updateTriggers() {
        this.intakeAccept.update();
        this.intakeReject.update();
    }

    public void senseColor() {
        if (!this.deposit.hasInnerPixel() || !this.deposit.hasOuterPixel()) {
            timer.reset();
        }

        if ((this.deposit.hasOuterPixel() && this.deposit.hasInnerPixel()) && !this.locked && timer.milliseconds() >= 200) {
            new LockInner(this.deposit).schedule();
            new LockOuter(this.deposit).schedule();
            this.locked = true;
        }
    }

    public Intake getIntake() {
        return this.intake;
    }

    public Deposit getDeposit() {
        return this.deposit;
    }

    public MecanumDrive getDrive() {
        return this.drive.drive;
    }

    public Type getType() {
        return this.type;
    }

    public enum Type {
        AUTON,
        TELEOP
    }
}
