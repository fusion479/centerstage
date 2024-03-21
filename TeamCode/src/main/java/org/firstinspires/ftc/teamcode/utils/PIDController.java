package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    private final double kP, kI, kD;
    private final ElapsedTime timer = new ElapsedTime();
    private double target = 0;
    private double integralSum = 0, lastError = 0;

    public PIDController(double kP) {
        this.kP = kP;
        this.kI = 0;
        this.kD = 0;
    }

    public PIDController(double kP, double kI) {
        this.kP = kP;
        this.kI = kI;
        this.kD = 0;
    }

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
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

        return (kP * error) + (kI * integralSum) + (kD * derivative);
    }

    public double getLastError() {
        return lastError;
    }

}

