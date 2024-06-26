package org.firstinspires.ftc.teamcode.opmodes.auton.red.close;

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
import org.firstinspires.ftc.teamcode.opmodes.auton.Trajectories;
import org.firstinspires.ftc.teamcode.subsystems.camera.Camera;
import org.firstinspires.ftc.teamcode.utils.CommandAction;

@Autonomous(name = "2+0 Blue Close", group = "_Auto")
public class TwoPlusZero extends CommandOpMode {
    private MultipleTelemetry multipleTelemetry;
    private CommandRobot robot;
    private Camera camera;

    @Override
    public void initialize() {
        this.multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = new CommandRobot(super.hardwareMap, new GamepadEx(this.gamepad1), new GamepadEx(this.gamepad2), this.multipleTelemetry, Positions.CLOSE.START, CommandRobot.Type.AUTON);
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

        Trajectories.Close CLOSE = new Trajectories(Camera.Color.RED, this.robot.getDrive()).new Close();
        Action initialPath = region == 1 ? CLOSE.LEFT_SPIKEMARK : this.camera.getRegion() == 2 ? CLOSE.MID_SPIKEMARK : CLOSE.RIGHT_SPIKEMARK;

        Actions.runBlocking(new ParallelAction(
                initialPath,
                new SequentialAction(
                        new CommandAction(new WaitCommand(5000)),
                        new CommandAction(this.robot.scoreLow),
                        new CommandAction(new WaitCommand(2000)),
                        new CommandAction(this.robot.scoreOne),
                        new CommandAction(new WaitCommand(500)),
                        new CommandAction(this.robot.ready),
                        new CommandAction(new WaitCommand(1000))
                )
        ));

        Actions.runBlocking(CLOSE.getPark());

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().disable();
        CommandScheduler.getInstance().reset();
    }
}