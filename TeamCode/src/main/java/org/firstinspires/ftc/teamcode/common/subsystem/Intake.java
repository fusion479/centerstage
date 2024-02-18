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
    public static double UP_POS = .9;
    public static double DOWN_POS = .27;
    public static double IDLE_POS = .7;
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

        idle();
    }

    public void update() {
        intakeLeft.setPosition(target);
        intakeRight.setPosition(1 - target);
        intake.setPower(power);
    }

    public void setPower(double power) {
        Intake.power = power;
    }

    public void up() {
        target = UP_POS;
    }

    public void idle() {
        target = IDLE_POS;
    }

    public void down() {
        target = DOWN_POS;
    }
}