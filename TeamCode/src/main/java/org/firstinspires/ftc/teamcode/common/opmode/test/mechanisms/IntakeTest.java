package org.firstinspires.ftc.teamcode.common.opmode.test.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.subsystem.Intake;

@TeleOp(name = "Intake Test", group = "testing")
@Config
public class IntakeTest extends LinearOpMode {
    Intake intake = new Intake();

    @Override
    public void runOpMode() throws InterruptedException {
        intake.init(hardwareMap);

        waitForStart();
        while(opModeIsActive()){
            if (gamepad1.a) {
                intake.up(); // zero in up position
            } else if (gamepad1.b) {
                intake.idle();
            } else if (gamepad1.x) {
                intake.down();
            } else if (gamepad1.right_trigger > 0) {
               intake.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0) {
                intake.setPower(-gamepad1.left_trigger);
            } else {
                intake.setPower(0);
            }

            intake.update();
        }
    }
}
