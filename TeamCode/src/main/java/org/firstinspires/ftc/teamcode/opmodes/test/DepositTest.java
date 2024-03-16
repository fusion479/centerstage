package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Deposit;

import org.firstinspires.ftc.teamcode.commands.deposit.DepositReady;
import org.firstinspires.ftc.teamcode.commands.deposit.DepositScore;


@TeleOp(name = "Deposit Test", group = "Test")
public class DepositTest extends CommandOpMode {
    private Deposit deposit;
    private GamepadEx gamepad;

    @Override
    public void initialize() {
        this.deposit = new Deposit(this.hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
        this.gamepad = new GamepadEx(gamepad1);

        this.gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new DepositScore(this.deposit));
        this.gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new DepositReady(this.deposit));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.initialize();

        super.waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            CommandScheduler.getInstance().run();
        }
    }
}
