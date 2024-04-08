package org.firstinspires.ftc.teamcode.subsystems;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Deposit extends Subsystem {
    public static double ACCEPTING_POS = 0.78;
    public static double READY_POS = 0.58;
    public static double SCORE_POS = 0.35;

    public static double LOCK_INNER = 0.85;
    public static double OPEN_INNER = 0.65;
    public static double LOCK_OUTER = 0.2;
    public static double OPEN_OUTER = 0.85;

    public static int THRESHOLD_MM = 15;
    public static double RED_GAIN = 0.2;
    public static double BLUE_GAIN = 0.2;
    public static double GREEN_GAIN = 0.2;


    private final Servo pivot, inner, outer;
    ColorRangeSensor innerSensor, outerSensor;
    LowPassFilter lowPassFilterR = new LowPassFilter(RED_GAIN);
    LowPassFilter lowPassFilterB = new LowPassFilter(BLUE_GAIN);
    LowPassFilter lowPassFilterG = new LowPassFilter(GREEN_GAIN);

    public Deposit(final HardwareMap hwMap, final MultipleTelemetry telemetry) {
        super(telemetry);

        this.pivot = hwMap.get(Servo.class, "depositPivot");
        this.inner = hwMap.get(Servo.class, "innerPixel");
        this.outer = hwMap.get(Servo.class, "outerPixel");
        innerSensor = hwMap.get(ColorRangeSensor.class, "colorInner");
        outerSensor = hwMap.get(ColorRangeSensor.class, "colorOuter");

        innerSensor.enableLed(false);
        outerSensor.enableLed(false);

        this.setPosition(1);
    }

    public void setPosition(double position) {
        this.pivot.setPosition(position);
    }

    public void setInnerPosition(double position) {
        this.inner.setPosition(position);
    }

    public void setOuterPosition(double position) {
        this.outer.setPosition(position);
    }

    public boolean hasPixel() {
//        return (innerSensor.red() > 200 && outerSensor.red() > 200);
        return (innerSensor.getDistance(DistanceUnit.MM) < THRESHOLD_MM) && (outerSensor.getDistance(DistanceUnit.MM) < THRESHOLD_MM);
    }

    public double redEstimate(ColorSensor s) {
        return getLowPass(s.red(), lowPassFilterR);
    }


    public double blueEstimate(ColorSensor s) {
        return getLowPass(s.blue(), lowPassFilterB);
    }

    public double greenEstimate(ColorSensor s) {
        return getLowPass(s.green(), lowPassFilterG);
    }

    public double getLowPass(double value, LowPassFilter filter) {
        return filter.estimate(value);
    }
}
