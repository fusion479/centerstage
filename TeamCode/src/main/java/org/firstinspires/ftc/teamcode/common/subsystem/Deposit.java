package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Deposit extends Mechanism {
    Servo depositLeft;
    Servo depositRight;
    public static double INTAKE_POS = 0.69;
    public static double SCORE_POS = 0.69;


    public void init(HardwareMap hwMap) {
        depositLeft = hwMap.get(Servo.class, "depositLeft");
        depositRight = hwMap.get(Servo.class, "depositRight");
    }

    public void setIntakePos() {
        depositLeft.setPosition(INTAKE_POS);
        depositRight.setPosition(1 - INTAKE_POS);
    }

    public void setScorePos() {
        depositLeft.setPosition(SCORE_POS);
        depositRight.setPosition(1 - SCORE_POS);
    }
}
