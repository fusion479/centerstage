package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    private final double kP, kI, kD;
    private final int allowedError;
    private final ElapsedTime timer = new ElapsedTime();
    private double target = 0;
    private double integralSum = 0, lastError = 0;

    public PIDController(double kP) {
        this.kP = kP;
        this.kI = 0;
        this.kD = 0;
        this.allowedError = 0;
    }

    public PIDController(double kP, double kI) {
        this.kP = kP;
        this.kI = kI;
        this.kD = 0;
        this.allowedError = 0;
    }

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.allowedError = 0;
    }

    public PIDController(double kP, double kI, double kD, int allowedError) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.allowedError = allowedError;
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

        if (isFinished()) {
            return 0;
        } else {
            return (kP * error) + (kI * integralSum) + (kD * derivative);
        }
    }

    public double getLastError() {
        return lastError;
    }

    public boolean isFinished() {
        return abs(lastError) <= allowedError;
    }

}

