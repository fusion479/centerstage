package org.firstinspires.ftc.teamcode.common.opmode.test.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.subsystem.Arm;
import org.firstinspires.ftc.teamcode.common.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystem.Intake;
import org.firstinspires.ftc.teamcode.common.subsystem.Lift;

@TeleOp(name = "Arm Test", group = "testing")
@Config
public class ArmTest extends LinearOpMode {
    Arm arm = new Arm();
    Lift lift = new Lift();
    Deposit deposit = new Deposit();
    Intake intake = new Intake();

    @Override
    public void runOpMode() throws InterruptedException {
        arm.init(hardwareMap);
        lift.init(hardwareMap);
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
                lift.medium();
            }else if (gamepad1.y) {
                lift.low();
            }

            arm.update();
            lift.update();
            deposit.update();
            intake.update();
            intake.setPower(gamepad1.right_trigger);
         }
    }
}
