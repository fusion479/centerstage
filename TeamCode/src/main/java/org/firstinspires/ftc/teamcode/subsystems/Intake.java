package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Subsystem;

@Config
public class Intake extends Subsystem {
    public static double ACCEPTING_POS = .68;
    public static double READY_POS = .9;
    public static double STACK_POS = .78;

    private final DcMotorEx intake;
    private final Servo intakeRight, intakeLeft;
    private double power;

    public Intake(final HardwareMap hwMap, final MultipleTelemetry telemetry) {
        super(telemetry);

        this.intakeLeft = hwMap.get(Servo.class, "intakeLeft");
        this.intakeRight = hwMap.get(Servo.class, "intakeRight");

        this.intake = hwMap.get(DcMotorEx.class, "intake");

        this.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.intake.setDirection(DcMotorEx.Direction.REVERSE);

        this.setPosition(0.9);
    }

    @Override
    public void periodic() {
        this.intake.setPower(power);
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void setPosition(double position) {
        intakeLeft.setPosition(position);
        intakeRight.setPosition(1 - position);
    }
}
