package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.util.Conversion;
import org.firstinspires.ftc.teamcode.common.util.PIDController;

public class Climber extends Mechanism {
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kG = 0;
    private final PIDController controller = new PIDController(kP, kI, kD);

    public static double target = 0;
    public static double power = 0;

    private DcMotorEx climber;
    private static final double WHEEL_RADIUS = 0;
    private static final double GEAR_RATIO = 0;
    private static final double TICKS_PER_REV = 145.1; // double check if correct ratio

    public MultipleTelemetry multipleTelemetry = new MultipleTelemetry();

    @Override
    public void init(HardwareMap hwMap) {
        climber = hwMap.get(DcMotorEx.class, "climber");

        climber.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        climber.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        climber.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void setTarget(int inches) {
        Lift.target = Conversion.inchesToTicks(inches, WHEEL_RADIUS, GEAR_RATIO, TICKS_PER_REV);
    }

    public void loop() {
        controller.setTarget(target);
        power = controller.calculate(climber.getCurrentPosition()) + kG;
        climber.setPower(power);

        multipleTelemetry.addData("target: ", target);
        multipleTelemetry.addData("current position: ", climber.getCurrentPosition());
    }

    public void toggle() {
        if (climber.getCurrentPosition() >= 0) {
            setTarget(0);
        } else {
            setTarget(20);
        }
    }
}
