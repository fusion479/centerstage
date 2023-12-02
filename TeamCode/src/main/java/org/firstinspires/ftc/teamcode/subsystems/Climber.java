package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Climber extends SubsystemBase {
    private final DcMotorEx motor;
    private final PIDFController controller;
    private double target;

    public Climber(final HardwareMap hwMap, final String name) {
        this.motor = hwMap.get(DcMotorEx.class, name);

        this.motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.motor.setDirection(DcMotorEx.Direction.FORWARD);

        this.controller = new PIDFController(0, 0, 0, 0);
    }


    @Override
    public void periodic() {
        double power = this.controller.calculate(this.motor.getCurrentPosition(), target);
        this.motor.setPower(power); // or set velocity
    }

    public void setTarget(double target) {
        this.target = target;
    }

}
