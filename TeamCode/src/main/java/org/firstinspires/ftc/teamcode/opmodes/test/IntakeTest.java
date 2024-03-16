package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

import org.firstinspires.ftc.teamcode.commands.intake.IntakeDown;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeIdle;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeSetPower;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeUp;


@TeleOp(name = "Intake Test", group = "Test")
public class IntakeTest extends CommandOpMode {
    private Intake intake;
    private GamepadEx gamepad;

    @Override
    public void initialize() {
        this.intake = new Intake(this.hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
        this.gamepad = new GamepadEx(gamepad1);

        this.gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new IntakeUp(this.intake));
        this.gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new IntakeDown(this.intake));
        this.gamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new IntakeIdle(this.intake));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.initialize();

        super.waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            CommandScheduler.getInstance().run();

            double rTrigValue = this.gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            double lTrigValue = this.gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

            if (rTrigValue > 0.05) {
                this.intake.setPower(rTrigValue);
            } else {
                this.intake.setPower(lTrigValue);
            }
        }
    }
}
