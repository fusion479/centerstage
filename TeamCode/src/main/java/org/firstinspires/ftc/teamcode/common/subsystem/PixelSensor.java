package org.firstinspires.ftc.teamcode.common.subsystem;

import android.graphics.Color;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PixelSensor extends Mechanism {

    private ColorRangeSensor sensor;
    private String name;

    private double far;

    public PixelSensor(LinearOpMode opMode, String name, double far) {
        this.opMode = opMode;
        this.name = name;
        this.far = far;
    }

    public void init(HardwareMap hwMap) {
        sensor = hwMap.get(ColorRangeSensor.class, name);

        sensor.enableLed(false);
    }

    public boolean isPixel() {
        Telemetry t = FtcDashboard.getInstance().getTelemetry();
        t.addData(name + " dist", sensor.getDistance(DistanceUnit.MM));
        t.update();
        return sensor.getDistance(DistanceUnit.MM) < far;
    }
}
