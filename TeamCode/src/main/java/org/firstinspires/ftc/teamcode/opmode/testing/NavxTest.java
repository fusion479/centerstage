package org.firstinspires.ftc.teamcode.opmode.testing;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.navx.AHRS;

@TeleOp (name = "NavX Test", group = "testing")
public class NavxTest extends LinearOpMode {
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    private AHRS navx_device;

    @Override
    public void runOpMode() throws InterruptedException {
        navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            double pitchAngleDegrees = navx_device.getPitch();
            double rollAngleDegrees = navx_device.getRoll();
            double yawAngleDegrees = navx_device.getYaw();

            telemetry.addData("yaw", yawAngleDegrees);
            telemetry.addData("pitch", pitchAngleDegrees);
            telemetry.addData("roll", rollAngleDegrees);
            telemetry.addData("temperature:", navx_device.getTempC());
            telemetry.update();
        }


    }
}
