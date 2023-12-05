package org.firstinspires.ftc.teamcode.common.opmode.testing.mechTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.subsystem.Arm;
import org.firstinspires.ftc.teamcode.common.subsystem.Lift;

@TeleOp(name = "Arm Test", group = "testing")
@Config
public class ArmTest extends LinearOpMode {
    Arm arm = new Arm();

    @Override
    public void runOpMode() throws InterruptedException {
        arm.init(hardwareMap);

        waitForStart();

        // Scan servo till stop pressed.
        while(opModeIsActive()){
            if (gamepad1.a) {
               arm.up();
            } else if (gamepad1.b) {
                arm.down();
            } else if (gamepad1.y) {
                arm.toggle();
            }
         }
    }
}
