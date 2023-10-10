package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {
    private final Servo servo;

    public static double LAUNCH_POS = 1;
    public static double IDLE_POS = 0;

    public Launcher(HardwareMap hwMap) {
        this.servo = hwMap.get(Servo.class, "launcher");
    }

    public void launch() {
        servo.setPosition(LAUNCH_POS);
    }

    public void reset() {
        servo.setPosition(IDLE_POS);
    }

}
