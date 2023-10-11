package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Deposit {
    Servo deposit;

    public static double INTAKE_POS = 0.69;
    public static double SCORE_POS = 0.69;

    public static double OPEN_POS = 0.69;
    public static double CLOSED_POS = 0.69;

    public Deposit(HardwareMap hwMap) {
        deposit = hwMap.get(Servo.class, "deposit");

        close(); // on init, close the deposit
    }

    public void open() {
        deposit.setPosition(OPEN_POS);
    }

    public void close() {
        deposit.setPosition(CLOSED_POS);
    }

}
