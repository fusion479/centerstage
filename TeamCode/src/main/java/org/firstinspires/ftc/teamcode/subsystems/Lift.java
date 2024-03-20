package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Lift extends Subsystem {
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kG = 0;
    public static double target = 0;
    private final PIDFController controller = new PIDFController(kP, kI, kD, kG);
    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
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
    }

    @Override
    public void periodic() {
        // this.power = this.controller.calculate(this.leftMotor.getCurrentPosition() + 1.03, target); // 0.03 is Kg
        this.leftMotor.setPower(power);
        this.rightMotor.setPower(power);
        super.getTelemetry().addLine("RJA");
    }

    public void setTarget(double target) {
        Lift.target = target;
    }

    public void setPower(double power) {
        this.power = power;
    }
}