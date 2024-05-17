package org.firstinspires.ftc.teamcode.opmodes.auton.red.close;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.arm.ArmScore;
import org.firstinspires.ftc.teamcode.commands.deposit.DepositScore;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.CommandGroupAction;

@Autonomous(name = "2+0 Red Close", group = "_Auto")
public class TwoPlusZero extends CommandOpMode {
    private MultipleTelemetry multipleTelemetry;
    private Drivetrain drive;
    private Lift lift;
    private Arm arm;
    private Deposit deposit;
    private Intake intake;

    @Override
    public void initialize() {
        this.multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.arm = new Arm(hardwareMap, this.multipleTelemetry);
        this.deposit = new Deposit(hardwareMap, this.multipleTelemetry);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().enable();
        this.initialize();

        super.waitForStart();

        while (!super.isStopRequested() && super.opModeIsActive()) {
            CommandScheduler.getInstance().run();

            Actions.runBlocking(
                    new CommandGroupAction(
                            new SequentialCommandGroup(
                                    new ArmScore(this.arm),
                                    new DepositScore(this.deposit)
                            )
                    )
            );

            this.multipleTelemetry.update();
        }

        CommandScheduler.getInstance().disable();
    }
}