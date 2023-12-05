package org.firstinspires.ftc.teamcode.common.opmode.testing.mechTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.subsystem.Lift;

@TeleOp(name = "Intake Test", group = "testing")
@Config
public class IntakeTest extends LinearOpMode {
    DcMotorEx motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "intake");

        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // might be wrong RunMode

        waitForStart();

        // Scan servo till stop pressed.
        while(opModeIsActive()){
            motor.setPower(gamepad1.left_stick_y);
        }
    }
}
