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
        isPressedY = gamepad1.y;  // medium
        isPressedA = gamepad1.a; // ready bottom
        isPressedB = gamepad1.b; // low
        isPressedLB = gamepad1.left_bumper; // toggle inner pixel
        isPressedRB = gamepad1.right_bumper; // toggle outer pixel

        if (!isPressedA && gamepad1.a) {
            lift.bottom();
            arm.down();
            deposit.accepting();
        } else if (!isPressedB && gamepad1.b) {
            lift.low();
            arm.up();
            deposit.ready();
        } else if (!isPressedX && gamepad1.x) {
            lift.medium();
            arm.up();
            deposit.ready();
        } else if (!isPressedY && gamepad1.y) {
            lift.high();
            arm.up();
            deposit.ready();
        } else if (!isPressedLB && gamepad1.left_bumper) {
            deposit.toggleOuter();
        } else if (!isPressedRB && gamepad1.right_bumper) {
            deposit.toggleInner();
        } else if (gamepad1.right_trigger > .1) {
            lift.bottom();
            arm.down();
            deposit.accepting();
        }

        intake.setPower(gamepad1.right_trigger);
        intake.setPower(-gamepad1.left_trigger);

        lift.update();
        arm.update();
        deposit.update();
        drive.update();
    }

}
