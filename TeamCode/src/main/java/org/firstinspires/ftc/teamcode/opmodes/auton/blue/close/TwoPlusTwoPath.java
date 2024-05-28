package org.firstinspires.ftc.teamcode.opmodes.auton.blue.close;

import static fi.iki.elonen.NanoWSD.WebSocketFrame.OpCode.Close;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.arm.ArmScore;
import org.firstinspires.ftc.teamcode.commands.deposit.DepositScore;
import org.firstinspires.ftc.teamcode.opmodes.auton.Positions;
import org.firstinspires.ftc.teamcode.opmodes.auton.Trajectories;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.camera.Camera;
import org.firstinspires.ftc.teamcode.utils.CommandGroupAction;


@Autonomous(name = "2+2 Blue Close Path", group = "_Auto")
public class TwoPlusTwoPath extends CommandOpMode {
    private MultipleTelemetry multipleTelemetry;
    private Drivetrain drive;
    private Lift lift;
    private Arm arm;
    private Deposit deposit;
    private Intake intake;
    private Camera camera;
    private Trajectories trajectories;
    private Trajectories.Close close;
    private Trajectories.General general;


    @Override
    public void initialize() {
        this.multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.camera = new Camera(Camera.Color.BLUE, this.multipleTelemetry);
        this.drive = new Drivetrain(hardwareMap, this.multipleTelemetry, Positions.CLOSE.START);
        this.trajectories = new Trajectories(Camera.Color.BLUE, this.drive);
        close = trajectories.new Close();
        general = trajectories.new General();


    }

    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler.getInstance().enable();
        this.initialize();

        super.waitForStart();

        while (!super.isStopRequested() && super.opModeIsActive()) {
            CommandScheduler.getInstance().run();
            int region = camera.getRegion();
            if (region == 1) {
                Actions.runBlocking(
                        close.LEFT_SPIKEMARK

                );
            } else if (region == 2) {
                Actions.runBlocking(
                        close.MID_SPIKEMARK
                );

            }
            else {
                Actions.runBlocking(
                        close.RIGHT_SPIKEMARK
                );
            }


            this.multipleTelemetry.update();
        }

        CommandScheduler.getInstance().disable();
    }
}