package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.PIDController;

@Config
public class Intake extends Mechanism {
    // PID Coefficients
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    private PIDController controller = new PIDController(kP, kI, kD);
    public static double target = 0; // RPM
    public static double power = 0;

    DcMotorEx intake;

    MultipleTelemetry multipleTelemetry = new MultipleTelemetry();

    @Override
    public void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotorEx.class, "intake");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void loop() {
        controller.setTarget(target);
        power = controller.calculate(intake.getVelocity());
        intake.setPower(power);

        multipleTelemetry.addData("current intake power", power);
    }

}