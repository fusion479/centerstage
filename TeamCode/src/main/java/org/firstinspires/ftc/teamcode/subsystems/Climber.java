package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Climber extends Mechanism {
    public static double UP_POS = 1;
    public static double DOWN_POS = 0;
    private boolean isUp;

    private Servo servo;

    @Override
    public void init(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "climb");
    }

    public void up() {
        servo.setPosition(UP_POS);
        isUp = true;
    }

    public void down() {
        servo.setPosition(DOWN_POS);
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
