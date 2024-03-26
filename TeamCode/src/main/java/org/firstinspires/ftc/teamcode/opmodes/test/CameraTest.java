package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.camera.Camera;

public class CameraTest extends CommandOpMode {
    private Camera camera;
    private MultipleTelemetry multipleTelemetry;

    @Override
    public void initialize() {
        this.multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.camera = new Camera(Camera.Color.BLUE, this.multipleTelemetry);

        this.camera.initCamera(super.hardwareMap);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.initialize();
        CommandScheduler.getInstance().enable();

        super.waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            CommandScheduler.getInstance().run();

            this.multipleTelemetry.addData("Region", this.camera.getRegion());

            this.multipleTelemetry.update();
        }

        CommandScheduler.getInstance().disable();
    }
}
