package org.firstinspires.ftc.teamcode.common.subsystem;

import android.graphics.Color;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class PixelSensor extends Mechanism {
    ColorRangeSensor innerSensor, outerSensor;
    public static int THRESHOLD_MM = 30;

    public static double RED_GAIN = 0.2;
    LowPassFilter lowPassFilterR = new LowPassFilter(RED_GAIN);

    public static double BLUE_GAIN = 0.2;
    LowPassFilter lowPassFilterB = new LowPassFilter(BLUE_GAIN);

    public static double GREEN_GAIN = 0.2;
    LowPassFilter lowPassFilterG = new LowPassFilter(GREEN_GAIN);

    @Override
    public void init(HardwareMap hwMap) {
        innerSensor = hwMap.get(ColorRangeSensor.class, "colorInner");
        outerSensor = hwMap.get(ColorRangeSensor.class, "colorOuter");

        innerSensor.enableLed(false);
        outerSensor.enableLed(false);
    }

    public boolean hasPixel() {
//        return (innerSensor.red() > 200 && outerSensor.red() > 200);
        return (innerSensor.getDistance(DistanceUnit.MM) < THRESHOLD_MM) && (outerSensor.getDistance(DistanceUnit.MM) < THRESHOLD_MM);
    }

    public double redEstimate(ColorSensor s){
        return getLowPass(s.red(), lowPassFilterR);
    }


    public double blueEstimate(ColorSensor s){
        return getLowPass(s.blue(), lowPassFilterB);
    }

    public double greenEstimate(ColorSensor s){
        return getLowPass(s.green(), lowPassFilterG);
    }

    public double getLowPass(double value, LowPassFilter filter) {
        return filter.estimate(value);
    }
}
