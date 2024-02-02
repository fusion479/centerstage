package org.firstinspires.ftc.teamcode.common.opmode.test.miscellaneous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.subsystem.Camera;

@TeleOp(name = "Red Camera Test", group = "testing")
@Config
public class RedCameraTest extends LinearOpMode {
    Camera camera = new Camera("red");
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        camera.init(hardwareMap);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Region", camera.whichRegion());
            telemetry.update();
        }
    }
}
