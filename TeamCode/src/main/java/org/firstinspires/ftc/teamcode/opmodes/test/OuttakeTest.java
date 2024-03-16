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

import org.firstinspires.ftc.teamcode.commands.arm.ArmReady;
import org.firstinspires.ftc.teamcode.commands.deposit.DepositAccepting;
import org.firstinspires.ftc.teamcode.commands.deposit.DepositIdle;
import org.firstinspires.ftc.teamcode.commands.deposit.DepositScore;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

import org.firstinspires.ftc.teamcode.commands.arm.ArmDown;
import org.firstinspires.ftc.teamcode.commands.arm.ArmUp;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;


@TeleOp(name = "Arm Test", group = "Test")
public class OuttakeTest extends CommandOpMode {
    private Arm arm;
    private Deposit deposit;
    private Lift lift;
    private GamepadEx gamepad;

    @Override
    public void initialize() {
        this.arm = new Arm(this.hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
        this.deposit = new Deposit(this.hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
        this.lift = new Lift(this.hardwareMap, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
        this.gamepad = new GamepadEx(gamepad1);

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
        this.initialize();

        super.waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            CommandScheduler.getInstance().run();
        }
    }
}
