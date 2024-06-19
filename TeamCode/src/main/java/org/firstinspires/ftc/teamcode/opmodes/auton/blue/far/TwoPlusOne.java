package org.firstinspires.ftc.teamcode.opmodes.auton.blue.far;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.example.meepmeeptesting.Positions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandRobot;
import org.firstinspires.ftc.teamcode.commands.auton.IntakeSetPower;
import org.firstinspires.ftc.teamcode.commands.auton.IntakeUntilPixel;
import org.firstinspires.ftc.teamcode.opmodes.auton.Trajectories;
import org.firstinspires.ftc.teamcode.subsystems.camera.Camera;
import org.firstinspires.ftc.teamcode.utils.CommandAction;

@Autonomous(name = "2+1 Blue Far", group = "_Auto")
public class TwoPlusOne extends CommandOpMode {
    private MultipleTelemetry multipleTelemetry;
    private CommandRobot robot;
    private Trajectories.Far FAR;
    private Trajectories.General GENERAL;
    private Camera camera;

    @Override
    public void initialize() {
        this.multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = new CommandRobot(super.hardwareMap, new GamepadEx(this.gamepad1), new GamepadEx(this.gamepad2), this.multipleTelemetry, Positions.FAR.START);
        this.camera = new Camera(Camera.Color.BLUE, this.multipleTelemetry);
        this.camera.initCamera(super.hardwareMap);

        this.FAR = new Trajectories(Camera.Color.BLUE, this.robot.getDrive()).new Far();
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

        Action initialPath = region == 1 ? this.FAR.LEFT_SPIKEMARK : region == 2 ? this.FAR.MID_SPIKEMARK : this.FAR.RIGHT_SPIKEMARK;

        this.camera.stopStreaming();
        Actions.runBlocking(new ParallelAction(
                initialPath,
                new SequentialAction(
                        new CommandAction(new WaitCommand(6250)),
                        new CommandAction(this.robot.stack),
                        new CommandAction(new IntakeUntilPixel(this.robot.getDeposit(), this.robot.getIntake()))
                )
        ));

        this.GENERAL = new Trajectories(Camera.Color.BLUE, this.robot.getDrive()).new General();
        Action stackToBackdrop = region == 1 ? this.GENERAL.STACK_TO_LEFT_BACKDROP : region == 2 ? this.GENERAL.STACK_TO_MID_BACKDROP : this.GENERAL.STACK_TO_RIGHT_BACKDROP;

        Actions.runBlocking(new ParallelAction(
                stackToBackdrop,
                new SequentialAction(
                        new CommandAction(new WaitCommand(250)),
                        new CommandAction(new IntakeSetPower(this.robot.getIntake(), 1, 1000)),
                        new CommandAction(new WaitCommand(6200)),
                        new CommandAction(this.robot.scoreLow),
                        new CommandAction(new WaitCommand(3000)),
                        new CommandAction(this.robot.scoreOne),
                        new CommandAction(this.robot.scoreTwo),
                        new CommandAction(new WaitCommand(1000))
                )
        ));

        Actions.runBlocking(this.FAR.getPark());

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().disable();
        CommandScheduler.getInstance().reset();
    }
}