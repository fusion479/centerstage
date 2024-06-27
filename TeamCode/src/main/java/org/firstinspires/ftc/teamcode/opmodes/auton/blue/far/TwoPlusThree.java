package org.firstinspires.ftc.teamcode.opmodes.auton.blue.far;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.example.meepmeeptesting.Positions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandRobot;
import org.firstinspires.ftc.teamcode.commands.auton.IntakeSetPower;
import org.firstinspires.ftc.teamcode.commands.auton.IntakeUntilPixel;
import org.firstinspires.ftc.teamcode.opmodes.auton.Trajectories;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.camera.Camera;
import org.firstinspires.ftc.teamcode.utils.CommandAction;

@Autonomous(name = "2+3 Blue Far", group = "_Auto")
public class TwoPlusThree extends CommandOpMode {
    private MultipleTelemetry multipleTelemetry;
    private CommandRobot robot;
    private Camera camera;

    @Override
    public void initialize() {
        this.multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = new CommandRobot(
                super.hardwareMap,
                new GamepadEx(this.gamepad1),
                new GamepadEx(this.gamepad2),
                this.multipleTelemetry,
                Positions.FAR.START,
                CommandRobot.Type.AUTON);
        this.camera = new Camera(Camera.Color.BLUE, this.multipleTelemetry);
        this.camera.initCamera(super.hardwareMap);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().enable();
        this.initialize();

        while (!super.isStarted()) {
            this.multipleTelemetry.addData("Region:", this.camera.getRegion());
            this.multipleTelemetry.update();
        }
        int region = this.camera.getRegion();
        this.camera.stopStreaming();

        Trajectories.Far FAR = new Trajectories(Camera.Color.BLUE, this.robot.getDrive()).new Far();
        Action initialPath = region == 1 ? FAR.LEFT_SPIKEMARK : region == 2 ? FAR.MID_SPIKEMARK : FAR.RIGHT_SPIKEMARK;
        int STACK_DELAY = region == 1 ? 2000 : region == 3 ? 3000 : 1000;
        int INTAKE_DURATION = region == 1 ? 60000 : region == 2 ? 5000 : 7000;

        Actions.runBlocking(new ParallelAction(
                initialPath,
                new SequentialAction(
                        new CommandAction(new WaitCommand(2500)),
                        new CommandAction(new InstantCommand(() -> this.robot.getIntake().setPosition(Intake.ACCEPTING_POS))),
                        new CommandAction(new WaitCommand(STACK_DELAY)),
                        new CommandAction(this.robot.stack),
                        new CommandAction(new IntakeUntilPixel(this.robot.getDeposit(), this.robot.getIntake(), INTAKE_DURATION)),
                        new CommandAction(new WaitCommand(250)),
                        new CommandAction(new IntakeSetPower(this.robot.getIntake(), 500, 1)),
                        new CommandAction(new InstantCommand(() -> this.robot.getIntake().setPosition(Intake.ACCEPTING_POS))) // don't interfere
                )
        ));

        Trajectories.General GENERAL = new Trajectories(Camera.Color.BLUE, this.robot.getDrive()).new General();
        Action stackToBackdrop = region == 1 ? GENERAL.STACK_TO_LEFT_BACKDROP(this.robot.getDrive().pose) : region == 2 ? GENERAL.STACK_TO_MID_BACKDROP(this.robot.getDrive().pose) : GENERAL.STACK_TO_RIGHT_BACKDROP(this.robot.getDrive().pose);

        Actions.runBlocking(new ParallelAction(
                stackToBackdrop,
                new SequentialAction(
                        new CommandAction(new WaitCommand(250)),
                        new CommandAction(new InstantCommand(() -> this.robot.getIntake().setPosition(Intake.ACCEPTING_POS + 0.05))),
                        new CommandAction(new WaitCommand(4000)),
                        new CommandAction(this.robot.scoreLow),
                        new CommandAction(new WaitCommand(2000)),
                        new CommandAction(this.robot.scoreOne),
                        new CommandAction(new WaitCommand(250)),
                        new CommandAction(this.robot.scoreBottom),
                        new CommandAction(new WaitCommand(500)),
                        new CommandAction(new InstantCommand(() -> this.robot.getDeposit().setOuterPosition(Deposit.OPEN_OUTER))),
                        new CommandAction(this.robot.scoreTwo),
                        new CommandAction(new WaitCommand(500))
                )
        ));

        Action backdropToStack = region == 1 ? GENERAL.LEFT_BACKDROP_TO_STACK(this.robot.getDrive().pose) : region == 2 ? GENERAL.MID_BACKDROP_TO_STACK(this.robot.getDrive().pose) : GENERAL.RIGHT_BACKDROP_TO_STACK(this.robot.getDrive().pose);

        Actions.runBlocking(new ParallelAction(
                backdropToStack,
                new SequentialAction(
                        new CommandAction(new WaitCommand(200)),
                        new CommandAction(this.robot.accepting),
                        new CommandAction(new WaitCommand(1000)),
                        new CommandAction(new IntakeUntilPixel(this.robot.getDeposit(), this.robot.getIntake(), 5000)),
                        new CommandAction(new WaitCommand(250)),
                        new CommandAction(new IntakeSetPower(this.robot.getIntake(), 500, 1)),
                        new CommandAction(new InstantCommand(() -> this.robot.getIntake().setPosition(Intake.ACCEPTING_POS))) // don't interfere
                )
        ));

        stackToBackdrop = region == 1 ? GENERAL.STACK_TO_LEFT_BACKDROP(this.robot.getDrive().pose) : region == 2 ? GENERAL.STACK_TO_MID_BACKDROP(this.robot.getDrive().pose) : GENERAL.STACK_TO_RIGHT_BACKDROP(this.robot.getDrive().pose);

        Actions.runBlocking(new ParallelAction(
                stackToBackdrop,
                new SequentialAction(
                        new CommandAction(new WaitCommand(250)),
                        new CommandAction(new InstantCommand(() -> this.robot.getIntake().setPosition(Intake.ACCEPTING_POS + 0.05))),
                        new CommandAction(new WaitCommand(3500)),
                        new CommandAction(this.robot.scoreLow),
                        new CommandAction(new WaitCommand(2000)),
                        new CommandAction(this.robot.scoreOne),
                        new CommandAction(new WaitCommand(250)),
                        new CommandAction(this.robot.scoreTwo),
                        new CommandAction(new WaitCommand(500))
                )
        ));

        Actions.runBlocking(FAR.getPark());

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().disable();
        CommandScheduler.getInstance().reset();
    }
}