package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.util.Conversion;
import org.firstinspires.ftc.teamcode.common.util.PIDController;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class Lift extends Mechanism {
    // PID Coefficients
    public static double kP = 0.03;
    public static double kI = 0;
    public static double kD = 0;
    public static double kG = 0.03;
    public static double target = 0; // maximum 700 ticks
    public static double power = 0;
    private final PIDController controller = new PIDController(kP, kI, kD);

    // Motor info declarations
    private final DcMotorEx[] motors = new DcMotorEx[2];
    private static final double WHEEL_RADIUS = 0.6738964567;
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REV = 145.1;

//     telemetry
    Telemetry tele;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry telemetry;

    @Override
    public void init(HardwareMap hwMap) {
        motors[0] = hwMap.get(DcMotorEx.class, "liftLeft");
        motors[1] = hwMap.get(DcMotorEx.class, "liftRight");

        motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        telemetry();
    }

    public void initTele(Telemetry tele) {
        this.tele = tele;
        telemetry = new MultipleTelemetry(tele, dashboard.getTelemetry());
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
        setTarget(200);
    }

    public void medium() {
        setTarget(400);
    }

    public void high() {
        setTarget(600);
    }

    public void setTarget(int target) {
        this.target = target;
    }

    public double getPosition() {
        return motors[0].getCurrentPosition();
    }
}
