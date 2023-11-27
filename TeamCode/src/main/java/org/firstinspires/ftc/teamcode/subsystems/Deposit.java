package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Deposit {
    Servo depositLeft;
    Servo depositRight;
    public static double INTAKE_POS = 0.69;
    public static double SCORE_POS = 0.69;

    public static double OPEN_POS = 0.69;
    
    public static double CLOSED_POS = 0.69;

    public Deposit(HardwareMap hwMap) {
        depositLeft = hwMap.get(Servo.class, "depositLeft");
        depositRight = hwMap.get(Servo.class, "depositRight");

        // close(); // on init, close the deposit
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
