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
    private boolean isUp;
    double power;

    @Override
    public void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void loop() {
        intake.setPower(power);
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void raise() {
        intakeLeft.setPosition(UP_POS);
        intakeRight.setPosition(1-UP_POS);
        isUp = true;
    }
    public void lower() {
        intakeLeft.setPosition(DOWN_POS);
        intakeRight.setPosition(1-DOWN_POS);
        isUp = false;
    }
    public void toggle() {
        if (isUp) {
            raise();
        }
        else {
            lower();
        }
    }






}