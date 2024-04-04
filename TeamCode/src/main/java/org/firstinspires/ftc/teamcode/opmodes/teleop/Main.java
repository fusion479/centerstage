package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandRobot;

@TeleOp(name = "Main", group = "TeleOp")
public class Main extends CommandOpMode {
    private CommandRobot robot;
    private MultipleTelemetry multipleTelemetry;

    public void initialize() {
        this.multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.robot = new CommandRobot(
                this.hardwareMap,
                new GamepadEx(gamepad1),
                new GamepadEx(gamepad2),
                new MultipleTelemetry(telemetry, this.multipleTelemetry));

        this.robot.configureCommands();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.enable();
        CommandScheduler.getInstance().enable();

        this.initialize();

        super.waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            CommandScheduler.getInstance().run();

            this.robot.updateTriggers();

            this.multipleTelemetry.update();
        }

        CommandScheduler.getInstance().disable();
        CommandScheduler.getInstance().cancelAll();
        Robot.disable();
        this.robot.reset();
    }
}
