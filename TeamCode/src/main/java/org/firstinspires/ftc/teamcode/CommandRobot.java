package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.InstantCommand;
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
import org.firstinspires.ftc.teamcode.commands.deposit.LockOuter;
import org.firstinspires.ftc.teamcode.commands.deposit.locks.LockInner;
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
    private final Deposit deposit;
    private final GamepadTrigger intakeAccept;
    private final GamepadTrigger intakeReject;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean autoLocked = false;

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
                .toggleWhenPressed(new SequentialCommandGroup(
                        new LowLift(this.lift),
                        new ArmAccepting(this.arm),
                        new DepositReady(this.deposit),
                        new IntakeAccepting(this.intake),
                        new OpenInner(this.deposit),
                        new OpenOuter(this.deposit),
                        new DepositAccepting(this.deposit),
                        new BottomLift(this.lift)
                ), new SequentialCommandGroup(
                        new LockInner(this.deposit),
                        new LockOuter(this.deposit),
                        new LowLift(this.lift),
                        new ArmReady(this.arm),
                        new DepositReady(this.deposit),
                        new IntakeReady(this.intake),
                        new BottomLift(this.lift)
                ));


        // LIFT LOW
        this.gamepad1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new SequentialCommandGroup(
                        new LockOuter(this.deposit),
                        new LockInner(this.deposit),
                        new LowLift(this.lift),
                        new ArmScore(this.arm),
                        new DepositScore(this.deposit),
                        new IntakeReady(this.intake)
                ));
        // LIFT HIGH
        this.gamepad1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SequentialCommandGroup(
                        new LockOuter(this.deposit),
                        new LockInner(this.deposit),
                        new HighLift(this.lift),
                        new ArmScore(this.arm),
                        new DepositScore(this.deposit),
                        new IntakeReady(this.intake)));

        // LIFT MEDIUM
        this.gamepad1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new SequentialCommandGroup(
                        new LockOuter(this.deposit),
                        new LockInner(this.deposit),
                        new MediumLift(this.lift),
                        new ArmScore(this.arm),
                        new DepositScore(this.deposit),
                        new IntakeReady(this.intake)));

        // LIFT UP A LITTLE
        this.gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new SequentialCommandGroup(
                        new LockOuter(this.deposit),
                        new LockInner(this.deposit),
                        new ArmScore(this.arm),
                        new DepositScore(this.deposit),
                        new IntakeReady(this.intake),
                        new LiftRaise(this.lift)));

        // LIFT DOWN A LITTLE
        this.gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SequentialCommandGroup(
                        new LockOuter(this.deposit),
                        new LockInner(this.deposit),
                        new ArmScore(this.arm),
                        new DepositScore(this.deposit),
                        new IntakeReady(this.intake),
                        new LiftLower(this.lift)));

        // SCORE PIXEL ONE
        this.gamepad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> this.deposit.setOuterPosition(Deposit.OPEN_OUTER)),
                        new WaitCommand(175),
                        new LiftRaise(this.lift),
                        new InstantCommand(() -> {
                            this.autoLocked = false;
                            this.timer.reset();
                        })
                ));

        // SCORE PIXEL TWO
        this.gamepad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new SequentialCommandGroup(
                        new OpenInner(this.deposit),
                        new InstantCommand(() -> this.autoLocked = false),
                        new WaitCommand(175),
                        new LiftRaise(this.lift),
                        new InstantCommand(() -> {
                            this.autoLocked = false;
                            this.timer.reset();
                        }
                        )));

        // LAUNCHER COMMANDS
        this.gamepad2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new Launch(this.launcher));
        this.gamepad2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new Idle(this.launcher));
    }

    public void updateTriggers() {
        this.intakeAccept.update();
        this.intakeReject.update();
    }

    public void senseColor() {
        if (this.deposit.hasPixel() && !this.autoLocked && timer.milliseconds() >= 300) {
            // new LockInner(this.deposit).schedule();
            // new LockOuter(this.deposit).schedule();
            this.deposit.setInnerPosition(Deposit.LOCK_INNER);
            this.deposit.setOuterPosition(Deposit.LOCK_OUTER);
            this.autoLocked = true;
        }
    }
}
