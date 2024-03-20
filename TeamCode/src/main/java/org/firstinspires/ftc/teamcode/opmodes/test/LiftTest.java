package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.lift.LowLift;
import org.firstinspires.ftc.teamcode.commands.lift.MediumLift;
import org.firstinspires.ftc.teamcode.subsystems.Lift;


@TeleOp(name = "Lift Test", group = "Test")
public class LiftTest extends CommandOpMode {
    private Lift lift;
    private GamepadEx gamepad;
    private MultipleTelemetry multipleTelemetry;

    @Override
    public void initialize() {
        this.multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.lift = new Lift(this.hardwareMap, this.multipleTelemetry);
        this.gamepad = new GamepadEx(gamepad1);

        this.gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new MediumLift(this.lift));
        this.gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new LowLift(this.lift));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().enable();
        this.initialize();

        super.waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            CommandScheduler.getInstance().run();

            this.multipleTelemetry.update();
        }

        CommandScheduler.getInstance().disable();
    }
}