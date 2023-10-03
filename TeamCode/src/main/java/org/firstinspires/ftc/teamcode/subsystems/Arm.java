package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    Servo left, right;
    private double setPos = 0.44;
    private boolean up;
    public void init(HardwareMap hwmap) {
        left = hwmap.get(Servo.class, "armLeft");
        right = hwmap.get(Servo.class, "armRight");
        up = false;
    }
    public void setUp() {
        left.setPosition(setPos);
        right.setPosition(setPos);
        up = true;
    }
    public void setDown() {
        left.setPosition(0);
        right.setPosition(0);
        up = false;
    }
    public void toggle() {
        if (up) {
            setDown();
        }
        else {
            setUp();
        }

    }
}
