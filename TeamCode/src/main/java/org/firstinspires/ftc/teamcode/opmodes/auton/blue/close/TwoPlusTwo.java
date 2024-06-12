package org.firstinspires.ftc.teamcode.opmodes.auton.blue.close;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.example.meepmeeptesting.Positions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandRobot;
import org.firstinspires.ftc.teamcode.commands.arm.ArmScore;
import org.firstinspires.ftc.teamcode.commands.deposit.DepositScore;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeSetPower;
import org.firstinspires.ftc.teamcode.opmodes.auton.Trajectories;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.camera.Camera;
import org.firstinspires.ftc.teamcode.utils.CommandAction;
import org.firstinspires.ftc.teamcode.utils.CommandGroupAction;

@Autonomous(name = "2+2 Blue Close", group = "_Auto")
public class TwoPlusTwo extends CommandOpMode {
    private MultipleTelemetry multipleTelemetry;
    private CommandRobot robot;
    private Trajectories.Close CLOSE;
    private Trajectories.General GENERAL;
    private Camera camera;

    @Override
    public void initialize() {
        this.multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = new CommandRobot(super.hardwareMap, new GamepadEx(this.gamepad1), new GamepadEx(this.gamepad2), this.multipleTelemetry, Positions.CLOSE.START);
        this.camera = new Camera(Camera.Color.BLUE, this.multipleTelemetry);

        this.CLOSE = new Trajectories(Camera.Color.BLUE, this.robot.getDrive()).new Close();
        this.GENERAL = new Trajectories(Camera.Color.BLUE, this.robot.getDrive()).new General();

    }

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().enable();
        this.initialize();

        Action initialPath;
        Action backdropToStack;
        Action stackToBackdrop;

        if (this.camera.getRegion() == 1) {
           initialPath = this.CLOSE.LEFT_SPIKEMARK;
           backdropToStack = this.GENERAL.LEFT_BACKDROP_TO_STACK;
           stackToBackdrop = this.GENERAL.STACK_TO_LEFT_BACKDROP;
        } else if (this.camera.getRegion() == 2) {
            initialPath = this.CLOSE.MID_SPIKEMARK;
            backdropToStack = this.GENERAL.MID_BACKDROP_TO_STACK;
            stackToBackdrop = this.GENERAL.STACK_TO_MID_BACKDROP;
        } else {
            initialPath = this.CLOSE.RIGHT_SPIKEMARK;
            backdropToStack = this.GENERAL.RIGHT_BACKDROP_TO_STACK;
            stackToBackdrop = this.GENERAL.STACK_TO_RIGHT_BACKDROP;
        }

        super.waitForStart();

        Actions.runBlocking(new ParallelAction(
                initialPath,
                new SequentialAction(
                        new CommandAction(new WaitCommand(5000)),
                        new CommandAction(this.robot.scoreLow),
                        new CommandAction(this.robot.scoreOne),
                        new CommandAction(new WaitCommand(2000)),
                        new CommandAction(this.robot.stack)
                ),
                backdropToStack,
                new ParallelAction(
                        new CommandAction(new WaitCommand(5000)),
                        new CommandAction(new IntakeSetPower(this.robot.intake, 0.5))
                ),
                stackToBackdrop,
                new SequentialAction(
                        new CommandAction(new WaitCommand(5000)),
                        new CommandAction(this.robot.scoreLow),
                        new CommandAction(this.robot.scoreTwo),
                        new CommandAction(new WaitCommand(2000)),
                        new CommandAction(this.robot.ready)
                ),
                this.CLOSE.getPark()
        ));

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().disable();
        CommandScheduler.getInstance().reset();
    }
}