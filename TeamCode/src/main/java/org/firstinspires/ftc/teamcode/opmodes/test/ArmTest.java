package org.firstinspires.ftc.teamcode.opmodes.test;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.arm.ArmDown;
import org.firstinspires.ftc.teamcode.commands.arm.ArmUp;
import org.firstinspires.ftc.teamcode.subsystems.Arm;


@TeleOp(name = "Arm", group = "Test")
public class ArmTest extends CommandOpMode {
    private Arm arm;
    private GamepadEx gamepad;
    @Override
    public void initialize() {
        this.arm = new Arm(this.hardwareMap);
        this.gamepad = new GamepadEx(new Gamepad());

        this.gamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ArmUp(this.arm));
        this.gamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new ArmDown(this.arm));
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
