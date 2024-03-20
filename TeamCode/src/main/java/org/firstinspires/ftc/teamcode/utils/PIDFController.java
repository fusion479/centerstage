package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController {

    private final double kP, kI, kD, kF;
    private final ElapsedTime timer = new ElapsedTime();
    private double target = 0;
    private double integralSum = 0, lastError = 0;

    public PIDFController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = 0;
    }

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public double getTarget() {
        return target;
    }

    public void setTarget(double target) {
        this.target = target;
        lastError = 0;
    }

    public double calculate(double reference) {
        double error = target - reference;
        double derivative = (error - lastError) / timer.seconds();
        integralSum = integralSum + (error * timer.seconds());

        lastError = error;
        timer.reset();

        return (kP * error) + (kI * integralSum) + (kD * derivative) + kF;
    }

    public double getLastError() {
        return lastError;
    }

}

