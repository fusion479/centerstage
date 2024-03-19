package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GamepadTrigger;


@TeleOp(name = "Manual Lift Test", group = "Test")
public class ManualLiftTest extends CommandOpMode {
    private Lift lift;
    private GamepadEx gamepad;
    private MultipleTelemetry multipleTelemetry;
    private GamepadTrigger raiseLift;

    @Override
    public void initialize() {
        this.multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.lift = new Lift(this.hardwareMap, this.multipleTelemetry);
        this.gamepad = new GamepadEx(gamepad1);

        this.gamepad.getGamepadButton(GamepadKeys.Button.X)
                .whileHeld(new InstantCommand(() -> this.lift.setPower(0.5)))
                .whenReleased(new InstantCommand(() -> this.lift.setPower(0)));

        this.raiseLift = new GamepadTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER, this.lift::setPower, this.gamepad, this.multipleTelemetry);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().enable();
        this.initialize();

        super.waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            CommandScheduler.getInstance().run();

            this.raiseLift.update();

            this.multipleTelemetry.update();
        }


        CommandScheduler.getInstance().disable();
    }
}
