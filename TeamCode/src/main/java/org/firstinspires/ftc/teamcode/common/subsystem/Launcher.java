package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Launcher extends Mechanism {
    public static double LAUNCH_POS = 1;
    public static double IDLE_POS = 0;
    Servo servo;
    double target;
    
    @Override
    public void init(HardwareMap hwMap) {
        this.servo = hwMap.get(Servo.class, "drone");

        idle();
    }

    public void update() {
        servo.setPosition(target);
    }

    public void launch() {
        target = LAUNCH_POS;
    }

    public void idle() {
        target = IDLE_POS;
    }

}
