package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.deposit.DepositReady;
import org.firstinspires.ftc.teamcode.commands.deposit.DepositScore;
import org.firstinspires.ftc.teamcode.commands.deposit.locks.LockInner;
import org.firstinspires.ftc.teamcode.commands.deposit.locks.LockOuter;
import org.firstinspires.ftc.teamcode.commands.deposit.locks.OpenInner;
import org.firstinspires.ftc.teamcode.commands.deposit.locks.OpenOuter;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;


@TeleOp(name = "Deposit Test", group = "Test")
public class DepositTest extends CommandOpMode {
    private Deposit deposit;
    private GamepadEx gamepad;
    private MultipleTelemetry multipleTelemetry;

    @Override
    public void initialize() {
        this.multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.deposit = new Deposit(this.hardwareMap, this.multipleTelemetry);
        this.gamepad = new GamepadEx(gamepad1);

        this.gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new DepositScore(this.deposit));
        this.gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new DepositReady(this.deposit));
        this.gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new OpenInner(this.deposit));
        this.gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new OpenOuter(this.deposit));
        this.gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new LockInner(this.deposit));
        this.gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new LockOuter(this.deposit));
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
