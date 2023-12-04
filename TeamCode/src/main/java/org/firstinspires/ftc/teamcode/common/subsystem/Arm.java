package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm extends Mechanism {
    Servo left, right;
    public static double UP_POS = 0.44;
    public static double DOWN_POS = 0;

    private boolean isUp;

    @Override
    public void init(HardwareMap hwMap) {
        left = hwMap.get(Servo.class, "armLeft");
        right = hwMap.get(Servo.class, "armRight");
        isUp = false;
    }

    public void up() {
        left.setPosition(UP_POS);
        right.setPosition(1 - UP_POS);
        isUp = true;
    }

    public void down() {
        left.setPosition(DOWN_POS);
        right.setPosition(1- DOWN_POS);
        isUp = false;
    }

    public void toggle() {
        if (isUp) {
            down();
        } else {
            up();
        }
    }
}
