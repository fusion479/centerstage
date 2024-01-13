package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.PIDController;


@Config
public class Lift extends Mechanism {
    // PID Coefficients
    public static double kP = 0.0025;
    public static double kI = 0;
    public static double kD = 0;
    public static double kG = 0.03;
    public static double target = 0;
    public static double power = 0;
    private final PIDController controller = new PIDController(kP, kI, kD);

    // slides heights
    public static int bottom = 0;
    public static int low = 600;
    public static int medium = 1200;
    public static int high = 1900;

    // Motor info declarations
    private final DcMotorEx[] motors = new DcMotorEx[2];
    private static final double WHEEL_RADIUS = 0.7969769685;
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REV = 537.7;

    // telemetry
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

        motors[0].setDirection(DcMotorEx.Direction.REVERSE);
        motors[1].setDirection(DcMotorEx.Direction.FORWARD);

        bottom();
    }

    public void update() {
        controller.setTarget(target);
        power = controller.calculate(getPosition()) + kG;
        motors[0].setPower(power);
        motors[1].setPower(-power);

//        telemetry.addData("Current Position: ", getPosition());
//        telemetry.addData("Error: ", controller.getLastError());
//        telemetry.addData("Target: ", target);
//        telemetry.addData("Power: ", power);
//        telemetry.update();
    }

    public void initTele(Telemetry tele) {
        this.tele = tele;
        telemetry = new MultipleTelemetry(tele, dashboard.getTelemetry());
    }

    public void bottom() {
        setTarget(bottom);
    }

    public void low() {
        setTarget(low);
    }

    public void medium() {
        setTarget(medium);
    }

    public void high() {
        setTarget(high);
    }

    public void setTarget(int target) {
        this.target = target;
    }

    public double getPosition() {
        return motors[0].getCurrentPosition();
    }

    public void upALittle() {
        target += 150;
    }

    public void downALittle() {
        target -= 150;
    }
}
