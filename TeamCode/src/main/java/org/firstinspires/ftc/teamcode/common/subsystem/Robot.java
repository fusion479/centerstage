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
    Intake intake = new Intake();

    public boolean isPressedX = false;
    public boolean isPressedY = false;
    public boolean isPressedA = false;
    public boolean isPressedB = false;
    public boolean isPressedRB = false;
    public boolean isPressedLB = false;

    @Override
    public void init(HardwareMap hwMap) {
        drive = new SampleMecanumDrive(hwMap);
        lift.init(hwMap);
        arm.init(hwMap);
        deposit.init(hwMap);
        intake.init(hwMap);
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

        isPressedX = gamepad1.x; // high
        isPressedY = gamepad1.y  // medium
        isPressedA = gamepad1.a; //
        isPressedB = gamepad1.b;
        isPressedLB = gamepad1.left_bumper;
        isPressedRB = gamepad1.right_bumper;

        if (!isPressedA && gamepad1.a) {
            lift.bottom();
            arm.up();
            deposit.idle();
        } else if (!isPressedB && gamepad1.b) {
            lift.bottom();
            arm.down();
            deposit.accepting();
        } else if (!isPressedX && gamepad1.x) {
            lift.high();
            arm.up();
            deposit.idle();
        } else if (!isPressedY && gamepad1.y) {
            lift.high();
            arm.up();
            deposit.score();
        }

        intake.setPower(gamepad1.right_trigger);
        intake.setPower(-gamepad1.left_trigger);

        lift.update();
        arm.update();
        deposit.update();
        intake.update();
        drive.update();
    }

}
