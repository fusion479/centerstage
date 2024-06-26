package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.Subsystem;

@Config
public class Lift extends Subsystem {
    public static int BOTTOM_POS = 0;
    public static int LOW_POS = 400;
    public static int MEDIUM_POS = 1000;
    public static int INCREMENT = 150;
    public static int HIGH_POS = 1600;
    public static int CLIMB_POS = 2000;
    public static double kP = 0.007;
    public static double kG = 0.04;
    public static int allowedError = 25;
    private final PIDController controller = new PIDController(kP, 0, 0, allowedError);
    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private boolean liftUp = false;
    private double power;

    public Lift(final HardwareMap hwMap, final MultipleTelemetry telemetry) {
        super(telemetry);
        this.leftMotor = hwMap.get(DcMotorEx.class, "liftLeft");
        this.rightMotor = hwMap.get(DcMotorEx.class, "liftRight");

        this.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.setTarget(BOTTOM_POS);
    }

    public boolean getLiftUp() {
        return this.liftUp;
    }

    @Override
    public void periodic() {
        this.power = this.controller.calculate(this.leftMotor.getCurrentPosition()) + kG;

        this.leftMotor.setPower(power);
        this.rightMotor.setPower(power);

        super.getTelemetry().addData("currPower", power);
        super.getTelemetry().addData("currPos", this.leftMotor.getCurrentPosition());
        super.getTelemetry().addData("targetPos", this.controller.getTarget());
        super.getTelemetry().addData("error", this.controller.getLastError());

        this.liftUp = this.leftMotor.getCurrentPosition() > 20;
    }

    public boolean isFinished() {
        return this.controller.isFinished();
    }

    public void setPower(double power) {
        this.power = power;
    }

    public double getTarget() {
        return this.controller.getTarget();
    }

    public void setTarget(double target) {
        this.controller.setTarget(target);
    }
}
