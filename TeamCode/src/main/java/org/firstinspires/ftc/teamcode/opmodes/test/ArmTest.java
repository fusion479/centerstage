package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.arm.ArmDown;
import org.firstinspires.ftc.teamcode.commands.arm.ArmUp;
import org.firstinspires.ftc.teamcode.subsystems.Arm;


@TeleOp(name = "Arm Test", group = "Test")
public class ArmTest extends CommandOpMode {
    private Arm arm;
    private GamepadEx gamepad;
    private MultipleTelemetry multipleTelemetry;

    @Override
    public void initialize() {
        this.multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.arm = new Arm(this.hardwareMap, this.multipleTelemetry);
        this.gamepad = new GamepadEx(gamepad1);
        this.gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ArmUp(this.arm));
        this.gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ArmDown(this.arm));
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

        CommandScheduler.getInstance().disable();
    }
}
