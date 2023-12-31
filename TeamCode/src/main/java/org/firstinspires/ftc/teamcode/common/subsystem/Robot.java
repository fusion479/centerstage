package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Robot extends Mechanism {
    SampleMecanumDrive drive;
    Lift lift = new Lift();
    Arm arm = new Arm();
    Deposit deposit = new Deposit();

    public boolean isPressedX = false;
    public boolean isPressedY = false;
    public boolean isPressedA = false;
    public boolean isPressedB = false;

    @Override
    public void init(HardwareMap hwMap) {
        drive = new SampleMecanumDrive(hwMap);
        lift.init(hwMap);
        arm.init(hwMap);
        deposit.init(hwMap);
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

        isPressedY = gamepad1.y;
        isPressedX = gamepad1.x;
        isPressedA = gamepad1.a;
        isPressedB = gamepad1.b;

        if (!isPressedA && gamepad1.a) {
            lift.bottom();
            arm.up();
            deposit.idle();
        } else if (!isPressedB && gamepad1.b) {
            lift.bottom();
            arm.down();
            deposit.intake();
        } else if (!isPressedX && gamepad1.x) {
            lift.high();
            arm.up();
            deposit.idle();
        } else if (!isPressedY && gamepad1.y) {
            lift.high();
            arm.up();
            deposit.score();
        }

        lift.loop();
        arm.loop();
        deposit.loop();
        drive.update();
    }

}
