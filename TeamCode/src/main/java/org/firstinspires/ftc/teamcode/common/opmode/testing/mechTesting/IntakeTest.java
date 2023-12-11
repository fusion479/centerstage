package org.firstinspires.ftc.teamcode.common.opmode.testing.mechTesting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
