package org.firstinspires.ftc.teamcode.common.opmode.testing.mechTesting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.subsystem.Arm;
import org.firstinspires.ftc.teamcode.common.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.common.subsystem.Intake;
import org.firstinspires.ftc.teamcode.common.subsystem.Lift;

@TeleOp(name = "Deposit Test", group = "testing")
@Config
public class DepositTest extends LinearOpMode {
    Deposit deposit = new Deposit();

    @Override
    public void runOpMode() throws InterruptedException {
        deposit.init(hardwareMap);

        waitForStart();

        // Scan servo till stop pressed.
        while(opModeIsActive() && !isStopRequested()){
            if (gamepad1.a) {
                deposit.accepting();
            } else if (gamepad1.b) {
                deposit.ready();
            } else if (gamepad1.x) {
                deposit.idle();
            }else if (gamepad1.y) {
                deposit.score();
            }

            deposit.update();
        }
    }
}
