package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
    private final Servo pivot, inner, outer;

    public Deposit(final HardwareMap hwMap, final MultipleTelemetry telemetry) {
        super(telemetry);

        this.pivot = hwMap.get(Servo.class, "depositPivot");
        this.inner = hwMap.get(Servo.class, "innerPixel");
        this.outer = hwMap.get(Servo.class, "outerPixel");

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
}
