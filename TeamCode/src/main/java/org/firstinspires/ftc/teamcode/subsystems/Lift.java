package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.*;

@Config
public class Lift {
    // PID Coefficients
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kG = 0;
    private PIDController controller = new PIDController(kP, kI, kD);
    public static double target = 0; // inches
    public static double power = 0;

    // Motor info declarations
    private final DcMotorEx[] motors = new DcMotorEx[2];
    private static final double WHEEL_RADIUS = 1.37795; // inches, placeholder values right now
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REV = 145.1;

    // telemetry
    public MultipleTelemetry multipleTelemetry = new MultipleTelemetry();

    public Lift(HardwareMap hwMap) {
        motors[0] = hwMap.get(DcMotorEx.class, "leftLift");
        motors[1] = hwMap.get(DcMotorEx.class, "rightLift");

        motors[0].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motors[1].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motors[0].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[1].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motors[0].setDirection(DcMotorEx.Direction.FORWARD);
        motors[1].setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void loop() {
        controller.setTarget(target);
        power = controller.calculate(motors[0].getCurrentPosition()) + kG;
        motors[0].setPower(power);
        motors[1].setPower(power);

        multipleTelemetry.addData("target: ", target);
        multipleTelemetry.addData("current position: ", motors[0].getCurrentPosition());
    }

    public void goBottom() {
        setTarget(0);
    }

    public void goLow() {
        setTarget(10);
    }

    public void goMedium() {
        setTarget(15);
    }

    public void goHigh() {
        setTarget(20);
    }

    public void setTarget(int inches) {
        Lift.target = Conversion.inchesToTicks(inches, WHEEL_RADIUS, GEAR_RATIO, TICKS_PER_REV);
    }
}
