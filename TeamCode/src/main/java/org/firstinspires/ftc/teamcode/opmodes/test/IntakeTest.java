package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.intake.IntakeDown;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeIdle;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeUp;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.GamepadTrigger;


@TeleOp(name = "Intake Test", group = "Test")
public class IntakeTest extends CommandOpMode {
    private Intake intake;
    private GamepadEx gamepad;
    private GamepadTrigger intakeAccept, intakeReject;
    private MultipleTelemetry multipleTelemetry;

    @Override
    public void initialize() {
        this.multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.intake = new Intake(this.hardwareMap, this.multipleTelemetry);
        this.gamepad = new GamepadEx(gamepad1);

        this.gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new IntakeUp(this.intake));
        this.gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new IntakeDown(this.intake));
        this.gamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new IntakeIdle(this.intake));

        this.intakeAccept = new GamepadTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER, this.intake::setPower, this.gamepad);
        this.intakeReject = new GamepadTrigger(GamepadKeys.Trigger.LEFT_TRIGGER, d -> this.intake.setPower(-d), this.gamepad);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().enable();
        this.initialize();

        super.waitForStart();
        while (!super.isStopRequested() && super.opModeIsActive()) {
            CommandScheduler.getInstance().run();

            this.intakeAccept.update();
            this.intakeReject.update();

            this.multipleTelemetry.update();
        }

        CommandScheduler.getInstance().disable();
    }
}
