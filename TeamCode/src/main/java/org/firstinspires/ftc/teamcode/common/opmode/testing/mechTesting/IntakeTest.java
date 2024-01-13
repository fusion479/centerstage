package org.firstinspires.ftc.teamcode.common.opmode.testing.mechTesting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
                intake.up();
            } else if (gamepad1.b) {
                intake.idle();
            } else if (gamepad1.x) {
                intake.intaking();
            } else if (gamepad1.right_bumper) {
                intake.upALittle();
            } else if (gamepad1.left_bumper) {
                intake.downALittle();
            }

            intake.update();
            intake.setPower(gamepad1.right_trigger);
        }
    }
}
