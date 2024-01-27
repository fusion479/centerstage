package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm extends Mechanism {
    public static double UP_POS = .04;
    public static double READY_POS = .4;
    public static double DOWN_POS = .6;
    public static double target = DOWN_POS;
    Servo left, right;
    private boolean isUp;

    @Override
    public void init(HardwareMap hwMap) {
        left = hwMap.get(Servo.class, "armLeft");
        right = hwMap.get(Servo.class, "armRight");
    }

    public void update() {
        left.setPosition(target);
        right.setPosition(1 - target);
    }

    public void up() {
        target = UP_POS;
    }

    public void ready() {
        target = READY_POS;
    }

    public void down() {
        target = DOWN_POS;
    }
}
