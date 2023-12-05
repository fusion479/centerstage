package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends SubsystemBase {
    private final Servo leftServo, rightServo;
    private static double UP_POS = 0.44;
    private static double DOWN_POS = 0;

    public Arm(HardwareMap hwMap) {
        this.leftServo = hwMap.get(Servo.class, "armLeft");
        this.rightServo = hwMap.get(Servo.class, "armRight");
    }

    public void up() {
        this.leftServo.setPosition(UP_POS);
        this.rightServo.setPosition(1 - UP_POS);
    }

    public void down() {
        this.leftServo.setPosition(DOWN_POS);
        this.rightServo.setPosition(1 - DOWN_POS);
    }
}
