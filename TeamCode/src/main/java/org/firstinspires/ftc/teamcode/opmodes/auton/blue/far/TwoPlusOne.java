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
import org.firstinspires.ftc.teamcode.commands.intake.IntakeSetPower;
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
        this.robot = new CommandRobot(super.hardwareMap, new GamepadEx(this.gamepad1), new GamepadEx(this.gamepad2), this.multipleTelemetry, Positions.CLOSE.START);
        this.camera = new Camera(Camera.Color.BLUE, this.multipleTelemetry);
        this.camera.initCamera(super.hardwareMap);

        this.FAR = new Trajectories(Camera.Color.BLUE, this.robot.getDrive()).new Far();
        this.GENERAL = new Trajectories(Camera.Color.BLUE, this.robot.getDrive()).new General();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().enable();
        this.initialize();

        while (!super.isStarted()) {
            this.multipleTelemetry.addData("Region:", this.camera.getRegion());
            this.multipleTelemetry.update();
        }

        Action initialPath = this.camera.getRegion() == 1 ? this.FAR.LEFT_SPIKEMARK : this.camera.getRegion() == 2 ? this.FAR.MID_SPIKEMARK : this.FAR.RIGHT_SPIKEMARK;

        this.camera.stopStreaming();
        Actions.runBlocking(new ParallelAction(
                initialPath,
                new SequentialAction(
                        new CommandAction(new WaitCommand(5000)),
                        new CommandAction(this.robot.stack),
                        new ParallelAction(
                                new CommandAction(new WaitCommand(2000)),
                                new CommandAction(new IntakeSetPower(this.robot.intake, 1))
                        ),
                        new CommandAction(this.robot.ready),
                        new CommandAction(new WaitCommand(1000)),
                        new CommandAction(this.robot.lock),
                        new CommandAction(new WaitCommand(1000)),
                        new ParallelAction(
                                new CommandAction(new WaitCommand(2000)),
                                new CommandAction(new IntakeSetPower(this.robot.intake, -1))
                        )
                ),
                new SequentialAction(
                        new CommandAction(new WaitCommand(10000)),
                        new CommandAction(this.robot.scoreLow),
                        new CommandAction(new WaitCommand(2000)),
                        new CommandAction(this.robot.scoreOne),
                        new CommandAction(this.robot.scoreTwo)
                )
        ));

        Actions.runBlocking(this.FAR.getPark());

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().disable();
        CommandScheduler.getInstance().reset();
    }
}