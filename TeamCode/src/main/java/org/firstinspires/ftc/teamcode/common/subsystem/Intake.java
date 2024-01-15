package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake extends Mechanism {

    DcMotorEx intake;
    Servo intakeLeft;
    Servo intakeRight;
    public static double UP_POS = 1;
    public static double DOWN_POS = 0;
    public static double IDLE_POS = .25;
    public static double CUSTOM_POS = .5;
    double power;
    double intakePos;

    public enum STATES {
        UP,
        IDLE,
        INTAKING
    };
    public STATES intakeState = STATES.UP;

    @Override
    public void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        intakeLeft = hwMap.get(Servo.class, "intakeLeft");
        intakeRight = hwMap.get(Servo.class, "intakeRight");

        up();
    }

    public void update() {
//        switch (intakeState) {
//            case UP:
//                intakePos = UP_POS;
//            case IDLE:
//                intakePos = IDLE_POS;
//            case INTAKING:
//                intakePos = DOWN_POS;
//        }
//
            intake.setPower(power);
//        intakeLeft.setPosition(intakePos);
//        intakeRight.setPosition(1 - intakePos);
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void up() {
        intakeLeft.setPosition(UP_POS);
        intakeRight.setPosition(1 - UP_POS);
    }

    public void idle() {
        intakeLeft.setPosition(IDLE_POS);
        intakeRight.setPosition(1 - IDLE_POS);
    }

    public void intaking() {
        intakeLeft.setPosition(DOWN_POS);
        intakeRight.setPosition(1 - DOWN_POS);
    }

    public void downALittle() {
        CUSTOM_POS = CUSTOM_POS - .05;
        intakeLeft.setPosition(CUSTOM_POS);
        intakeRight.setPosition(1 - CUSTOM_POS);
    }

    public void upALittle() {
        CUSTOM_POS = CUSTOM_POS - .05;
        intakeLeft.setPosition(CUSTOM_POS);
        intakeRight.setPosition(1 - CUSTOM_POS);
    }
}