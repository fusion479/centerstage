package org.firstinspires.ftc.teamcode.common.opmode.test.mechanisms;

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
        lift.init(hardwareMap);
        lift.initTele(telemetry);

        waitForStart();
        while (opModeIsActive()) {
            lift.update();
            if (gamepad1.a) {
                lift.bottom();
            } else if (gamepad1.b) {
                lift.low();
            } else if (gamepad1.x) {
                lift.medium();
            } else if (gamepad1.y) {
                lift.high();
            } else if (gamepad1.right_bumper) {
                lift.upALittle();
            } else if (gamepad1.left_bumper) {
                lift.downALittle();
            }
            multipleTelemetry.update();
        }
    }
}
