package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.arm.ArmReady;
import org.firstinspires.ftc.teamcode.commands.arm.ArmScore;
import org.firstinspires.ftc.teamcode.commands.deposit.DepositReady;
import org.firstinspires.ftc.teamcode.commands.deposit.DepositScore;
import org.firstinspires.ftc.teamcode.commands.deposit.locks.LockInner;
import org.firstinspires.ftc.teamcode.commands.deposit.locks.LockOuter;
import org.firstinspires.ftc.teamcode.commands.deposit.locks.OpenInner;
import org.firstinspires.ftc.teamcode.commands.deposit.locks.OpenOuter;
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
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class Commands {
    public Command ready, intake, scoreLow, scoreHigh, scoreMid, liftRaise, liftLower, scoreOne, scoreTwo, launch, launchIdle;

    public Commands(Arm arm, Lift lift, Launcher launcher, Intake intake, Deposit deposit, ElapsedTime timer, Runnable autoLockFalse, Runnable resetTimer) {
        this.ready = new SequentialCommandGroup(new LockInner(deposit),
                new LockOuter(deposit),
                new LowLift(lift),
                new ArmReady(arm),
                new DepositReady(deposit),
                new IntakeReady(intake),
                new BottomLift(lift));

        this.intake = new SequentialCommandGroup(
                new LockInner(deposit),
                new LockOuter(deposit),
                new LowLift(lift),
                new ArmReady(arm),
                new DepositReady(deposit),
                new IntakeReady(intake),
                new BottomLift(lift));

        this.scoreLow = new ParallelCommandGroup(
                new LockOuter(deposit),
                new LockInner(deposit),
                new LowLift(lift),
                new ArmScore(arm),
                new DepositScore(deposit),
                new IntakeReady(intake)
        );

        this.scoreMid = new ParallelCommandGroup(
                new LockOuter(deposit),
                new LockInner(deposit),
                new MediumLift(lift),
                new ArmScore(arm),
                new DepositScore(deposit),
                new IntakeReady(intake));

        this.scoreHigh = new ParallelCommandGroup(
                new LockOuter(deposit),
                new LockInner(deposit),
                new HighLift(lift),
                new ArmScore(arm),
                new DepositScore(deposit),
                new IntakeReady(intake));

        this.liftRaise = new SequentialCommandGroup(
                new LockOuter(deposit),
                new LockInner(deposit),
                new ArmScore(arm),
                new DepositScore(deposit),
                new IntakeReady(intake),
                new LiftRaise(lift));

        this.liftLower = new SequentialCommandGroup(
                new LockOuter(deposit),
                new LockInner(deposit),
                new ArmScore(arm),
                new DepositScore(deposit),
                new IntakeReady(intake),
                new LiftLower(lift));

        this.scoreOne = new SequentialCommandGroup(
                new OpenOuter(deposit),
                new WaitCommand(175),
                new LiftRaise(lift),
                new InstantCommand(() -> {
                    autoLockFalse.run();
                    resetTimer.run();
                }));

        this.scoreTwo = new SequentialCommandGroup(
                new OpenInner(deposit),
                new InstantCommand(autoLockFalse),
                new WaitCommand(175),
                new LiftRaise(lift),
                new InstantCommand(() -> {
                    autoLockFalse.run();
                    resetTimer.run();
                }));

        this.launch = new Launch(launcher);
        this.launchIdle = new Idle(launcher);
    }
}
