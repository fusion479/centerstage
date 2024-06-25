package org.firstinspires.ftc.teamcode.opmodes.auton.red.far;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.example.meepmeeptesting.Positions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandRobot;
import org.firstinspires.ftc.teamcode.commands.auton.IntakeSetPower;
import org.firstinspires.ftc.teamcode.commands.auton.IntakeUntilPixel;
import org.firstinspires.ftc.teamcode.opmodes.auton.Trajectories;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.camera.Camera;
import org.firstinspires.ftc.teamcode.utils.CommandAction;
import org.firstinspires.ftc.teamcode.utils.Wait;

@Autonomous(name = "2+1 Red Far", group = "_Auto")
public class TwoPlusOne extends CommandOpMode {
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
        this.camera = new Camera(Camera.Color.RED, this.multipleTelemetry);
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

        Trajectories.Far FAR = new Trajectories(Camera.Color.RED, this.robot.getDrive()).new Far();
        Action initialPath = region == 1 ? FAR.LEFT_SPIKEMARK : region == 2 ? FAR.MID_SPIKEMARK : FAR.RIGHT_SPIKEMARK;

        Actions.runBlocking(new ParallelAction(
                initialPath,
                new SequentialAction(
                        new CommandAction(new Wait(4000)),
                        new CommandAction(new InstantCommand(() -> this.robot.getIntake().setPosition(Intake.ACCEPTING_POS))),
                        new CommandAction(new Wait(500)),
                        new CommandAction(this.robot.stack),
                        new CommandAction(new IntakeUntilPixel(this.robot.getDeposit(), this.robot.getIntake())),
                        new CommandAction(new Wait(250)),
                        new CommandAction(new IntakeSetPower(this.robot.getIntake(), 500)),
                        new CommandAction(new InstantCommand(() -> this.robot.getIntake().setPosition(Intake.ACCEPTING_POS))) // don't interfere
                )
        ));



        Trajectories.General GENERAL = new Trajectories(Camera.Color.RED, this.robot.getDrive()).new General();
        Action stackToBackdrop = region == 1 ? GENERAL.STACK_TO_LEFT_BACKDROP : region == 2 ? GENERAL.STACK_TO_MID_BACKDROP : GENERAL.STACK_TO_RIGHT_BACKDROP;

        Actions.runBlocking(new ParallelAction(
                stackToBackdrop,
                new SequentialAction(
                        new CommandAction(new Wait(6250)),
                        new CommandAction(this.robot.scoreLow),
                        new CommandAction(new Wait(2000)),
                        new CommandAction(this.robot.scoreOne),
                        new CommandAction(new Wait(250)),
                        new CommandAction(this.robot.scoreTwo),
                        new CommandAction(new Wait(1000))
                )
        ));

        Actions.runBlocking(FAR.getPark());

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().disable();
        CommandScheduler.getInstance().reset();
    }
}