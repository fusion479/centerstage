package org.firstinspires.ftc.teamcode.common.opmode.test.miscellaneous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.subsystem.Camera;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Auto April Tag Test", group = "_Auto")
@Config
public class AutoAprilTagTest extends LinearOpMode {
    public static int TAG_ID = 2;
    Camera camera = new Camera("blue");
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        camera.aprilTagInit(hardwareMap, TAG_ID);
        camera.setManualExposure(6, 250, isStopRequested(), telemetry, this);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if (camera.detectAprilTag(tele)) {
                camera.moveRobot(drive, tele);
                camera.relocalize(drive);
            } else {
                drive.setMotorPowers(0, 0, 0, 0);
            }

            drive.update();
            tele.update();
        }
    }
}
