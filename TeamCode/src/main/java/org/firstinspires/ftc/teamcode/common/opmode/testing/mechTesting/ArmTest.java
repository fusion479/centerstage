package org.firstinspires.ftc.teamcode.common.opmode.testing.mechTesting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.subsystem.Arm;
import org.firstinspires.ftc.teamcode.common.subsystem.Deposit;

@TeleOp(name = "Arm Test", group = "testing")
@Config
public class ArmTest extends LinearOpMode {
    Arm arm = new Arm();
    Deposit deposit = new Deposit();

    @Override
    public void runOpMode() throws InterruptedException {
        arm.init(hardwareMap);
        deposit.init(hardwareMap);

        waitForStart();

        // Scan servo till stop pressed.
        while(opModeIsActive() && !isStopRequested()){
            if (gamepad1.a) {
               arm.up();
            } else if (gamepad1.b) {
                arm.down();
            } else if (gamepad1.x) {
                deposit.accepting();
            }else if (gamepad1.y) {
                deposit.idle();
            }
            arm.update();
            deposit.update();
         }
    }
}
