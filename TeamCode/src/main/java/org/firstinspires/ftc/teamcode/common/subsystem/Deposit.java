package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Deposit extends Mechanism {
    Servo pivot, inner, outer;
    public static double ACCEPTING_POS = .2;
    public static double IDLE_POS = .5;
    public static double SCORE_POS = .4;

    public static double LOCK = .7;
    public static double OPEN = .0;
    public static double pivotTarget = ACCEPTING_POS;
    public static double innerTarget = OPEN;
    public static double outerTarget = OPEN;

    public static boolean innerLocked = false;
    public static boolean outerLocked = false;


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
        openInner();
        openOuter();
    }

    public void idle() {
        pivotTarget = IDLE_POS;
        lockInner();
        lockOuter();
    }

    public void ready() {
        pivotTarget = SCORE_POS;
        lockInner();
        lockOuter();
    }

    public void score() {
        pivotTarget = SCORE_POS;
        openInner();
        openOuter();
    }

    public void toggleInner() {
        if (innerLocked) {
            openInner();
        } else {
            lockInner();
        }
    }

    public void toggleOuter() {
        if (innerLocked) {
            openOuter();
        } else {
            lockOuter();
        }
    }

    public void lockInner() {
        innerTarget = LOCK;
        innerLocked = true;
    }

    public void lockOuter() {
        outerTarget = LOCK;
        outerLocked = true;
    }

    public void openInner() {
        innerTarget = OPEN;
        innerLocked = false;
    }

    public void openOuter() {
        outerTarget = OPEN;
        outerLocked = false;
    }
}
