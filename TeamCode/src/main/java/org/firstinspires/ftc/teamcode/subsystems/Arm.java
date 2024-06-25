package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Subsystem;

@Config
public class Arm extends Subsystem {
    public static double SCORE_POS = 0;
    public static double READY_POS = 0.66;
    public static double ACCEPTING_POS = 0.75;

    private final Servo leftServo, rightServo;

    public Arm(final HardwareMap hwMap, final MultipleTelemetry telemetry) {
        super(telemetry);

        this.leftServo = hwMap.get(Servo.class, "armLeft");
        this.rightServo = hwMap.get(Servo.class, "armRight");

        this.setPosition(0.42);
    }

    public void setPosition(double position) {
        this.leftServo.setPosition(position);
        this.rightServo.setPosition(1 - position);
    }
}
