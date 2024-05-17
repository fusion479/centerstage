package org.firstinspires.ftc.teamcode.subsystems;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Subsystem;

@Config
public class Deposit extends Subsystem {
    public static double ACCEPTING_POS = 0.68;
    public static double READY_POS = 0.45;
    public static double SCORE_POS = 0.18;

    public static double LOCK_INNER = 0.85;
    public static double OPEN_INNER = 0.65;
    public static double LOCK_OUTER = 0.2;
    public static double OPEN_OUTER = 0.85;

    public static int THRESHOLD_MM = 15;
    public static double RED_GAIN = 0.2;
    public static double BLUE_GAIN = 0.2;
    public static double GREEN_GAIN = 0.2;


    private final Servo pivot, inner, outer;
    private final ColorRangeSensor innerSensor;
    private final ColorRangeSensor outerSensor;
    private final LowPassFilter lowPassFilterR = new LowPassFilter(RED_GAIN);
    private final LowPassFilter lowPassFilterB = new LowPassFilter(BLUE_GAIN);
    private final LowPassFilter lowPassFilterG = new LowPassFilter(GREEN_GAIN);

    public Deposit(final HardwareMap hwMap, final MultipleTelemetry telemetry) {
        super(telemetry);

        this.pivot = hwMap.get(Servo.class, "depositPivot");
        this.inner = hwMap.get(Servo.class, "innerPixel");
        this.outer = hwMap.get(Servo.class, "outerPixel");

        innerSensor = hwMap.get(ColorRangeSensor.class, "colorInner");
        outerSensor = hwMap.get(ColorRangeSensor.class, "colorOuter");

        innerSensor.enableLed(false);
        outerSensor.enableLed(false);

        this.setPosition(0.1);
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
        return (innerSensor.getDistance(DistanceUnit.MM) < THRESHOLD_MM) && (outerSensor.getDistance(DistanceUnit.MM) < THRESHOLD_MM);
    }
}
