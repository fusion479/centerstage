package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher extends SubsystemBase {
    private final Servo servo;

    public Launcher(HardwareMap hwMap) {
        this.servo = hwMap.get(Servo.class, "launcher");
    }

    public void launch() {
        this.servo.setPosition(1);
    }

    public void idle() {
        this.servo.setPosition(0);
    }
}