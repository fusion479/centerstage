package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.navx.AHRS;

public class NavxTest extends OpMode {
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    private AHRS navx_device;

    @Override
    public void init() {
        navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);
    }

    @Override
    public void loop() {
        double pitchAngleDegrees = navx_device.getPitch();
        double rollAngleDegrees = navx_device.getRoll();
        double yawAngleDegrees = navx_device.getYaw();

        telemetry.addData("yaw", yawAngleDegrees);
        telemetry.addData("pitch", pitchAngleDegrees);
        telemetry.addData("roll", rollAngleDegrees);
    }

    @Override
    public void stop() {
        navx_device.close();
    }
}
