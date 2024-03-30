package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm extends Mechanism {
    public static double SCORE_POS = 0;
    public static double READY_POS = 0.61;
    public static double MAX_SCORE_POS = .05;
    public static double STACK_TOP_POS = 0.85;
    public static double DOWN_POS = 0.69;

    public static double AUTO_INIT_POS = .64;
    public static double target = DOWN_POS;
    Servo left, right;

    @Override
    public void init(HardwareMap hwMap) {
        left = hwMap.get(Servo.class, "armLeft");
        right = hwMap.get(Servo.class, "armRight");
    }

    public void update() {
        left.setPosition(target);
        right.setPosition(1 - target);
    }

    public void score() {
        target = SCORE_POS;
    }

    public void autoInit() {
        target = AUTO_INIT_POS;
    }

    public void ready() {
        target = READY_POS;
    }

    public void down() {
        target = DOWN_POS;
    }

    public void maxScore() {
        target = MAX_SCORE_POS;
    }

    public void stack() {
        target = STACK_TOP_POS;
    }
}
