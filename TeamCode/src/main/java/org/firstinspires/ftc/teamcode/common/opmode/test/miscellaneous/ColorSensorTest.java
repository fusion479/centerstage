package org.firstinspires.ftc.teamcode.common.opmode.test.miscellaneous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ColorSensorTest extends LinearOpMode {
    ColorSensor outer, inner;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {
        outer = hardwareMap.get(ColorSensor.class, "colorOuter");
        inner = hardwareMap.get(ColorSensor.class, "colorInner");

        outer.enableLed(false);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            tele.addData("outer red", outer.red());
            tele.addData("outer green", outer.green());
            tele.addData("outer blue", outer.blue());

            tele.addData("inner red", inner.red());
            tele.addData("inner green", inner.green());
            tele.addData("inner blue", inner.blue());
            tele.update();
        }
    }
}
