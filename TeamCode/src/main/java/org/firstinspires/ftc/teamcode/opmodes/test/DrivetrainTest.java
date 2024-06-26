package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;


@TeleOp(name = "Drivetrain Test", group = "Test")
public class DrivetrainTest extends CommandOpMode {
    private Drivetrain drive;
    private GamepadEx gamepad;
    private MultipleTelemetry multipleTelemetry;

    @Override
    public void initialize() {
        this.multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.initialize();
        CommandScheduler.getInstance().enable();

        super.waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            CommandScheduler.getInstance().run();

            this.multipleTelemetry.update();
        }

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().disable();
        CommandScheduler.getInstance().reset();
    }
}