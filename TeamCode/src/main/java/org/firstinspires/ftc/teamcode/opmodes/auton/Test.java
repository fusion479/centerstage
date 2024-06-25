package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.example.meepmeeptesting.Positions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandRobot;
import org.firstinspires.ftc.teamcode.commands.auton.IntakeUntilPixel;
import org.firstinspires.ftc.teamcode.utils.CommandAction;
import org.firstinspires.ftc.teamcode.utils.Wait;

@Autonomous(name = "Testing", group = "_Auto")
public class Test extends CommandOpMode {
    private MultipleTelemetry multipleTelemetry;
    private CommandRobot robot;

    @Override
    public void initialize() {
        this.multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = new CommandRobot(
                super.hardwareMap,
                new GamepadEx(this.gamepad1),
                new GamepadEx(this.gamepad2),
                this.multipleTelemetry,
                Positions.FAR.START,
                CommandRobot.Type.AUTON);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().enable();
        this.initialize();

        super.waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        new CommandAction(new IntakeUntilPixel(this.robot.getDeposit(), this.robot.getIntake()))
                )
        );

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().disable();
        CommandScheduler.getInstance().reset();
    }
}