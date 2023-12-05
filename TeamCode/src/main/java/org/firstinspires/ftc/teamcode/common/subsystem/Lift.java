package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.util.Conversion;
import org.firstinspires.ftc.teamcode.common.util.PIDController;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class Lift extends Mechanism {
    // PID Coefficients
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kG = 0;
    public static double target = 0; // inches
    public static double power = 0;
    private final PIDController controller = new PIDController(kP, kI, kD);

    // Motor info declarations
    private final DcMotorEx[] motors = new DcMotorEx[2];
    private static final double WHEEL_RADIUS = 0.6738964567;
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REV = 145.1;

    // telemetry
    Telemetry tele;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry telemetry = new MultipleTelemetry(tele, dashboard.getTelemetry());

    @Override
    public void init(HardwareMap hwMap) {
        motors[0] = hwMap.get(DcMotorEx.class, "leftLift");
        motors[1] = hwMap.get(DcMotorEx.class, "rightLift");

        motors[0].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // might be wrong RunMode
        motors[1].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

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
    }

    public void telemetry() {
        telemetry.addData("Current Position: ", motors[0].getCurrentPosition());
        telemetry.addData("Target: ", target);
        telemetry.addData("Power: ", power);
        telemetry.update();
    }

    public void bottom() {
        setTarget(0);
    }

    public void low() {
        setTarget(10);
    }

    public void medium() {
        setTarget(15);
    }

    public void high() {
        setTarget(20);
    }

    public void setTarget(int inches) {
        Lift.target = Conversion.inchesToTicks(inches, WHEEL_RADIUS, GEAR_RATIO, TICKS_PER_REV);
    }
}
