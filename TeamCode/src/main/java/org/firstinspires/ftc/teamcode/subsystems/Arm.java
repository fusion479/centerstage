package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm extends Subsystem {
    public static double UP_POS = 0.05;
    public static double READY_POS = 0.61;
    public static double DOWN_POS = 0.7;
    public static double CLIMB_POS = 0.26;
    public static double AUTO_INIT_POS = .6;
    private final Servo leftServo, rightServo;

    public Arm(final HardwareMap hwMap, final MultipleTelemetry telemetry) {
        super(telemetry);

        this.leftServo = hwMap.get(Servo.class, "armLeft");
        this.rightServo = hwMap.get(Servo.class, "armRight");

        this.setPosition(Arm.DOWN_POS);
    }

    public void setPosition(double position) {
        this.leftServo.setPosition(position);
        this.rightServo.setPosition(1 - position);
    }
}
