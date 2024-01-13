package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.util.Conversion;
import org.firstinspires.ftc.teamcode.common.util.PIDController;

public class Climber extends Mechanism {
    public static double LOCK_POS = .5;
    public static double RELEASE_POS = 1;
    public static double ODO_RETRACT_POS = .4;
    public static double ODO_DOWN_POS = .7;

    public Servo climberLeft;
    public Servo climberRight;
    public Servo odoLift;

    @Override
    public void init(HardwareMap hwMap) {
        climberLeft = hwMap.get(Servo.class, "climberLeft");
        climberRight = hwMap.get(Servo.class, "climberRight");
        odoLift = hwMap.get(Servo.class, "odoLift");
    }

    public void update() {

    }

    public void lock() {
        climberLeft.setPosition(LOCK_POS);
        climberRight.setPosition(1 - LOCK_POS);
    }

    public void release() {
        climberLeft.setPosition(RELEASE_POS);
        climberRight.setPosition(1 - RELEASE_POS);
    }

    public void odoDown() {
        odoLift.setPosition(ODO_DOWN_POS);
    }

    public void odoUp() {
        odoLift.setPosition(ODO_RETRACT_POS);
    }

}
