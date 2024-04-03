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
    public static double ACCEPTING_POS = .1;
    public static double IDLE_POS = .81;
    public static double READY_POS = 0.5;
    public static double SCORE_POS = 0.82;
    public static double AUTO_INIT_POS = 1;
    public static double AUTO_STACK_POS = 0.25;
    public static double LOCKINNER = 0.85;
    public static double OPENINNER = 0.65;
    public static double LOCKOUTER = 0.45;
    public static double OPENOUTER = 0.85;
    ColorRangeSensor innerSensor, outerSensor;
    public static int THRESHOLD_MM = 15;

    public static double RED_GAIN = 0.2;
    LowPassFilter lowPassFilterR = new LowPassFilter(RED_GAIN);

    public static double BLUE_GAIN = 0.2;
    LowPassFilter lowPassFilterB = new LowPassFilter(BLUE_GAIN);

    public static double GREEN_GAIN = 0.2;
    LowPassFilter lowPassFilterG = new LowPassFilter(GREEN_GAIN);
    private final Servo pivot, inner, outer;

    public Deposit(final HardwareMap hwMap, final MultipleTelemetry telemetry) {
        super(telemetry);

        this.pivot = hwMap.get(Servo.class, "depositPivot");
        this.inner = hwMap.get(Servo.class, "innerPixel");
        this.outer = hwMap.get(Servo.class, "outerPixel");
        innerSensor = hwMap.get(ColorRangeSensor.class, "colorInner");
        outerSensor = hwMap.get(ColorRangeSensor.class, "colorOuter");

        innerSensor.enableLed(false);
        outerSensor.enableLed(false);

        this.setPosition(Deposit.IDLE_POS);
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
