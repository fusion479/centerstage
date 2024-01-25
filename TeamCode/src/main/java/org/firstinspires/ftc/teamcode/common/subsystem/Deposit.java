package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Deposit extends Mechanism {
    Servo pivot, inner, outer;
    public static double ACCEPTING_POS = .15;
    public static double IDLE_POS = .5;
    public static double READY_POS = .4;
    public static double SCORE_POS = 1;

    public static double LOCKINNER = .7;
    public static double OPENINNER = .0;
    public static double LOCKOUTER = 0;
    public static double OPENOUTER = .7;
    public static double pivotTarget = ACCEPTING_POS;
    public static double innerTarget = OPENINNER;
    public static double outerTarget = OPENINNER;

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
        if (innerLocked) {
            openOuter();
        } else {
            lockOuter();
        }
    }

    public void lockInner() {
        inner.setPosition(LOCKINNER);
        innerLocked = true;
    }

    public void lockOuter() {
        outer.setPosition(LOCKOUTER);
        outerLocked = true;
    }

    public void openInner() {
        inner.setPosition(OPENINNER);
        innerLocked = false;
    }

    public void openOuter() {
        outer.setPosition(OPENOUTER);
        outerLocked = false;
    }
}
