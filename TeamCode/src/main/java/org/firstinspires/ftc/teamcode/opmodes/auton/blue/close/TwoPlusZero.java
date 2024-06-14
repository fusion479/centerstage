package org.firstinspires.ftc.teamcode.opmodes.auton.blue.close;

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
    private Trajectories.Close TRAJECTORIES;
    private Camera camera;

    @Override
    public void initialize() {
        this.multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = new CommandRobot(super.hardwareMap, new GamepadEx(this.gamepad1), new GamepadEx(this.gamepad2), this.multipleTelemetry, Positions.CLOSE.START);
        this.camera = new Camera(Camera.Color.BLUE, this.multipleTelemetry);

        this.TRAJECTORIES = new Trajectories(Camera.Color.BLUE, this.robot.getDrive()).new Close();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().enable();
        this.initialize();

        Action determinedPath = this.camera.getRegion() == 1 ? this.TRAJECTORIES.LEFT_SPIKEMARK : this.camera.getRegion() == 2 ? this.TRAJECTORIES.MID_SPIKEMARK : this.TRAJECTORIES.RIGHT_SPIKEMARK;

        super.waitForStart();

        Actions.runBlocking(new ParallelAction(
                determinedPath,
                new SequentialAction(
                        new CommandAction(new WaitCommand(5000)),
                        new CommandAction(this.robot.scoreLow),
                        new CommandAction(this.robot.scoreOne),
                        new CommandAction(new WaitCommand(2000)),
                        new CommandAction(this.robot.ready)
                )
        ));

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().disable();
        CommandScheduler.getInstance().reset();
    }
}