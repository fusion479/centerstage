package org.firstinspires.ftc.teamcode.common.opmode.testing.mechTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.subsystem.Lift;

@TeleOp(name = "Lift Test", group = "testing")
@Config
public class LiftTest extends LinearOpMode {
    private final DcMotorEx[] motors = new DcMotorEx[2];
//    Lift lift = new Lift();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


    @Override
    public void runOpMode() throws InterruptedException {
        motors[0] = hardwareMap.get(DcMotorEx.class, "leftLift");
        motors[1] = hardwareMap.get(DcMotorEx.class, "rightLift");

        motors[0].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // might be wrong RunMode
        motors[1].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motors[0].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[1].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motors[0].setDirection(DcMotorEx.Direction.FORWARD);
        motors[1].setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        // Scan servo till stop pressed.
        while(opModeIsActive()){
            motors[0].setPower(gamepad1.left_stick_y);
            motors[1].setPower(gamepad1.right_stick_y);

            multipleTelemetry.addData("current position: ", motors[0].getCurrentPosition());
            multipleTelemetry.update();
        }
    }
}
