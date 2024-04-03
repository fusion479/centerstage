package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandRobot;

@TeleOp(name = "Main", group = "TeleOp")
public class Main extends CommandOpMode {
    private CommandRobot robot;
    private FtcDashboard dashboard;

    public void initialize() {
        dashboard = FtcDashboard.getInstance();
        this.robot = new CommandRobot(
                this.hardwareMap,
                new GamepadEx(gamepad1),
                new GamepadEx(gamepad2),
                new MultipleTelemetry(telemetry, dashboard.getTelemetry()));

        Robot.enable();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        super.waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            this.robot.configureCommands();
            this.robot.intakeTriggers();
            this.robot.run(); // runs the command scheduler
            this.telemetry.update();
        }

        this.robot.reset();
        Robot.disable();
    }
}
