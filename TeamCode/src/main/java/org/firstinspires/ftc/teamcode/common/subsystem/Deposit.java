package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Deposit extends Mechanism {
    Servo left, right, cover;
    public static double INTAKE_POS = .2;
    public static double IDLE_POS = .5;
    public static double SCORE_POS = .4;
    public static double COVER_BLOCK = .7;
    public static double COVER_OPEN = .0;
    public static double target = INTAKE_POS;
    public static double coverTarget = COVER_OPEN;



    public void init(HardwareMap hwMap) {
        left = hwMap.get(Servo.class, "depositLeft");
        right = hwMap.get(Servo.class, "depositRight");
        cover = hwMap.get(Servo.class, "depositCover");
    }

    public void loop() {
        cover.setPosition(coverTarget);
        left.setPosition(target);
        right.setPosition(1 - target);
    }

    public void intake() {
        coverTarget = COVER_OPEN;
        target = INTAKE_POS;
    }

    public void idle() {
        coverTarget = COVER_BLOCK;
        target = IDLE_POS;
    }

    public void ready() {
        coverTarget = COVER_BLOCK;
        target = SCORE_POS;
    }

    public void score() {
        coverTarget = COVER_OPEN;
        target = SCORE_POS;
    }
}
