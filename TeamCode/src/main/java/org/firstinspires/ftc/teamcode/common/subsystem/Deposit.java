package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Deposit extends Mechanism {
    public static double ACCEPTING_POS = 0.83;
    public static double READY_POS = 0.58;
    public static double SCORE_POS = 0.35;
    public static double MAX_SCORE_POS = .2;

    public static double AUTO_INIT_POS = 0;
    public static double STACK_TOP_POS = 0.7;

    public static double LOCK_INNER = 0.5;
    public static double LOCK_OUTER = .3;

    public static double OPEN_INNER = 0.3;
    public static double OPEN_OUTER = 0.5;

    public static double pivotTarget = ACCEPTING_POS;
    public static double innerTarget = OPEN_INNER;
    public static double outerTarget = OPEN_INNER;

    public boolean innerLocked = false;
    public boolean outerLocked = false;

    Servo pivot, inner, outer;

    public void init(HardwareMap hwMap) {
        pivot = hwMap.get(Servo.class, "depositPivot");
        inner = hwMap.get(Servo.class, "innerPixel");
        outer = hwMap.get(Servo.class, "outerPixel");
    }

    public void update() {
        pivot.setPosition(pivotTarget);
        inner.setPosition(innerTarget);
        outer.setPosition(outerTarget);
    }

    public void accepting() {
        pivotTarget = ACCEPTING_POS;
    }

    public void ready() {
        pivotTarget = READY_POS;
    }

    public void score() {
        pivotTarget = SCORE_POS;
    }

    public void maxScore() {
        pivotTarget = MAX_SCORE_POS;
    }

    public void autoInit() {
        pivotTarget = AUTO_INIT_POS;
    }

    public void stack() {
        pivotTarget = STACK_TOP_POS;
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
        innerTarget = LOCK_INNER;
        innerLocked = true;
    }

    public void lockOuter() {
        outerTarget = LOCK_OUTER;
        outerLocked = true;
    }

    public void openInner() {
        innerTarget = OPEN_INNER;
        innerLocked = false;
    }

    public void openOuter() {
        outerTarget = OPEN_OUTER;
        outerLocked = false;
    }
}
