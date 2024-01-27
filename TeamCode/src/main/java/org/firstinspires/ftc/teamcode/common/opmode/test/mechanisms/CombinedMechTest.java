package org.firstinspires.ftc.teamcode.common.opmode.test.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.subsystem.Arm;
import org.firstinspires.ftc.teamcode.common.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystem.Intake;

@TeleOp(name = "Combined Mech Test", group = "testing")

public class CombinedMechTest extends LinearOpMode {

    Arm arm = new Arm();
    Deposit deposit = new Deposit();
    Intake intake = new Intake();

    @Override
    public void runOpMode() throws InterruptedException {
        arm.init(hardwareMap);
        deposit.init(hardwareMap);
        intake.init(hardwareMap);

        waitForStart();

        // Scan servo till stop pressed.
        while(opModeIsActive() && !isStopRequested()){
            if (gamepad1.a) {
                arm.up();
            } else if (gamepad1.b) {
                arm.down();
            } else if (gamepad1.x) {
                intake.intaking();
            }else if (gamepad1.y) {
                intake.idle();
            } else if (gamepad1.dpad_up) {
                deposit.score();
            } else if(gamepad1.dpad_down) {
                deposit.accepting();
            }

            arm.update();
            deposit.update();
            intake.update();
            intake.setPower(gamepad1.right_trigger);
        }
    }
}
