package org.firstinspires.ftc.teamcode.common.opmode.testing.mechTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.subsystem.Lift;

@TeleOp(name = "Lift Test", group = "testing")
@Config
public class LiftTest extends LinearOpMode {
//    private final DcMotorEx[] motors = new DcMotorEx[2];
    Lift lift = new Lift();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


    @Override
    public void runOpMode() throws InterruptedException {
//        motors[0] = hardwareMap.get(DcMotorEx.class, "liftLeft");
//        motors[1] = hardwareMap.get(DcMotorEx.class, "liftRight");
//
//        motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motors[0].setDirection(DcMotorSimple.Direction.FORWARD);
//        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);

        lift.init(hardwareMap);
        lift.initTele(telemetry);

        waitForStart();
        while(opModeIsActive()) {
//            motors[0].setPower(gamepad1.left_stick_y);
//            motors[1].setPower(gamepad1.left_stick_y);
////
//            multipleTelemetry.addData("current position: ", motors[0].getCurrentPosition());
            lift.update();
            if (gamepad1.a) {
                lift.bottom();
            } else if (gamepad1.b) {
                lift.low();
            } else if (gamepad1.x) {
                lift.medium();
            } else if (gamepad1.y) {
                lift.high();
            }
            multipleTelemetry.addData("current pos: ", lift.getPosition());
            multipleTelemetry.update();
        }
    }
}
