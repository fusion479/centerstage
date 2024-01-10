package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake extends Mechanism {

    DcMotorEx intake;
    Servo intakeLeft;
    Servo intakeRight;
    public static double UP_POS;
    public static double DOWN_POS;
    public static double IDLE_POS;
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
        intake.setDirection(DcMotorEx.Direction.FORWARD);

        intakeLeft = hwMap.get(Servo.class, "intakeLeft");
        intakeRight = hwMap.get(Servo.class, "intakeRight");

        intakeLeft.setPosition(UP_POS);
        intakeRight.setPosition(1 - UP_POS);
    }

    public void update() {
        switch (intakeState) {
            case UP:
                intakePos = UP_POS;
            case IDLE:
                intakePos = IDLE_POS;
            case INTAKING:
                intakePos = DOWN_POS;
        }

        intake.setPower(power);
        intakeLeft.setPosition(intakePos);
        intakeRight.setPosition(1 - intakePos);
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void up() {
        intakeState = STATES.UP;
    }

    public void idle() {
        intakeState = STATES.IDLE;
    }

    public void intaking() {
        intakeState = STATES.INTAKING;
    }
}