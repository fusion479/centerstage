package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.subsystems.Camera;

@TeleOp
@Config
public class CameraTest extends LinearOpMode {
    Camera camera = new Camera();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {
        camera.init(hardwareMap);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();
        }
    }
}
