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
    public static double UP_POS = .75;
    public static double DOWN_POS = .22;
    public static double IDLE_POS = .5;
    public static double CUSTOM_POS = .5;
    public static double target = UP_POS;
    public static double power;

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
        intakeLeft.setPosition(target);
        intakeRight.setPosition(1 - target);
        intake.setPower(power);
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void up() {
        target = UP_POS;
    }

    public void idle() {
        target = IDLE_POS;
    }

    public void intaking() {
        target = DOWN_POS;
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