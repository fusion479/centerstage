package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.arm.ArmDown;
import org.firstinspires.ftc.teamcode.commands.arm.ArmReady;
import org.firstinspires.ftc.teamcode.commands.arm.ArmUp;
import org.firstinspires.ftc.teamcode.commands.deposit.DepositAccepting;
import org.firstinspires.ftc.teamcode.commands.deposit.DepositIdle;
import org.firstinspires.ftc.teamcode.commands.deposit.DepositScore;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.GamepadTrigger;


@TeleOp(name = "Outtake Test", group = "Test")
public class OuttakeTest extends CommandOpMode {
    private Arm arm;
    private Deposit deposit;
    private Lift lift;
    private GamepadEx gamepad;
    private MultipleTelemetry multipleTelemetry;
    private GamepadTrigger raiseLift;
    private GamepadTrigger lowerLift;

    @Override
    public void initialize() {
        this.multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.arm = new Arm(this.hardwareMap, this.multipleTelemetry);
        this.deposit = new Deposit(this.hardwareMap, this.multipleTelemetry);
        this.lift = new Lift(this.hardwareMap, this.multipleTelemetry);
        this.gamepad = new GamepadEx(gamepad1);

        this.raiseLift = new GamepadTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER, this.lift::setPower, this.gamepad);
        this.lowerLift = new GamepadTrigger(GamepadKeys.Trigger.LEFT_TRIGGER, p -> this.lift.setPower(-p), this.gamepad);

        this.configureCommands();
    }

    public void configureCommands() {
        this.gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ParallelCommandGroup(new ArmUp(this.arm), new DepositScore(this.deposit)));
        this.gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ParallelRaceGroup(new ArmReady(this.arm), new DepositIdle(this.deposit)));
        this.gamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new ParallelCommandGroup(new ArmDown(this.arm), new DepositAccepting(this.deposit)));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().enable();
        this.initialize();

        super.waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            CommandScheduler.getInstance().run();

            this.raiseLift.update();
            this.lowerLift.update();

            this.multipleTelemetry.update();
        }

        CommandScheduler.getInstance().disable();
    }
}
