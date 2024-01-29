package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.PIDController;


@Config
public class Lift extends Mechanism {
    private static final double WHEEL_RADIUS = 0.7969769685;
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REV = 537.7;

    // PID Coefficients
    public static double kP = 0.0025;
    public static double kI = 0;
    public static double kD = 0;
    public static double kG = 0.03;
    public static double target = 0;
    public static double power = 0;
    public static double error = 0;
    public static double bound = 50;
    // slides heights
    public static int bottom = 0;
    public static int low = 800;
    public static int medium = 1600;
    public static int high = 2300;
    private final PIDController controller = new PIDController(kP, kI, kD);
    // Motor info declarations
    public final DcMotorEx[] motors = new DcMotorEx[2];
    public boolean isReached = false;

    ElapsedTime timer = new ElapsedTime();
    public static double bottomMotorOffDelay = 2000;

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

        motors[0].setDirection(DcMotorEx.Direction.FORWARD);
        motors[1].setDirection(DcMotorEx.Direction.REVERSE);

        bottom();
    }

    public void update() {
        controller.setTarget(target);
        error = getPosition() - target;
        power = Range.clip(controller.calculate(getPosition()), -1, 1) + kG;

        if (Math.abs(error) < bound) {
            power = kG;
            if (target == 0) {
                power = 0;
            }
            isReached = true;
        }

        if (timer.milliseconds() > bottomMotorOffDelay && target == 0) {
            power = 0;
        }


        motors[0].setPower(power);
        motors[1].setPower(power);

//        telemetry.addData("Current Position", getPosition());
//        telemetry.addData("Error", error);
//        telemetry.addData("Target", target);
//        telemetry.addData("Power", power);
//        telemetry.update();
    }

    public void initTele(Telemetry tele) {
        this.tele = tele;
        telemetry = new MultipleTelemetry(tele, dashboard.getTelemetry());
    }

    public void bottom() {
        setTarget(bottom);
        timer.reset();
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
        Lift.target = target;
    }

    public double getPosition() {
        return motors[0].getCurrentPosition();
    }
}
