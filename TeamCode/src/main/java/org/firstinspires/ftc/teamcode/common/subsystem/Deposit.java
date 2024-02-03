package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Deposit extends Mechanism {
    public static double ACCEPTING_POS = 0;
    public static double IDLE_POS = .6;
    public static double READY_POS = 1;
    public static double SCORE_POS = .78;
    public static double LOCKINNER = 0.85;
    public static double OPENINNER = 0.4;
    public static double LOCKOUTER = 0.45;
    public static double OPENOUTER = 0;
    public static double pivotTarget = ACCEPTING_POS;
    public static double innerTarget = OPENINNER;
    public static double outerTarget = OPENINNER;
    public boolean innerLocked = false;
    public boolean outerLocked = false;
    Servo pivot, inner, outer;

    public void init(HardwareMap hwMap) {
        pivot = hwMap.get(Servo.class, "depositPivot");
        inner = hwMap.get(Servo.class, "innerPixel");
        outer = hwMap.get(Servo.class, "outerPixel");

        idle();
    }

    public void update() {
        pivot.setPosition(pivotTarget);
        inner.setPosition(innerTarget);
        outer.setPosition(outerTarget);
    }

    public void accepting() {
        pivotTarget = ACCEPTING_POS;
    }

    public void idle() {
        pivotTarget = IDLE_POS;
    }

    public void ready() {
        pivotTarget = READY_POS;
    }

    public void score() {
        pivotTarget = SCORE_POS;
    }

    public void toggleInner() {
        if (innerLocked) {
            openInner();
        } else {
            lockInner();
        }
    }

    public void toggleOuter() {
        if (outerLocked) {
            openOuter();
        } else {
            lockOuter();
        }
    }

    public void lockInner() {
        innerTarget = LOCKINNER;
        innerLocked = true;
    }

    public void lockOuter() {
        outerTarget = LOCKOUTER;
        outerLocked = true;
    }

    public void openInner() {
        innerTarget = OPENINNER;
        innerLocked = false;
    }

    public void openOuter() {
        outerTarget = OPENOUTER;
        outerLocked = false;
    }
}
