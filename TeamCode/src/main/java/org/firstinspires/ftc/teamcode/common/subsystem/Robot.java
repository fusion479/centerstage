package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Robot extends Mechanism {
    SampleMecanumDrive drive;
//    Lift lift = new Lift();
    Arm arm = new Arm();
    Deposit deposit = new Deposit();
    Intake intake = new Intake();
//    Launcher launcher = new Launcher();

    public boolean isPressedX = false;
    public boolean isPressedY = false;
    public boolean isPressedA = false;
    public boolean isPressedB = false;
    public boolean isPressedRB = false;
    public boolean isPressedLB = false;
    public boolean isPressedDPadDown = false;
    public boolean isPressedDPadUp = false;

    @Override
    public void init(HardwareMap hwMap) {
        drive = new SampleMecanumDrive(hwMap);
//        lift.init(hwMap);
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
        isPressedDPadUp = gamepad1.dpad_up;
        isPressedDPadDown = gamepad1.dpad_down;

        if (!isPressedA && gamepad1.a) {
            arm.down();
            deposit.accepting();
        } else if (!isPressedDPadDown && gamepad1.dpad_down) {
//            launcher.launch();
        } else if (!isPressedDPadUp && gamepad1.dpad_up) {
//            launcher.idle();
        } else if (!isPressedX && gamepad1.x) {
            arm.up();
            deposit.ready();
        } else if (!isPressedY && gamepad1.y) {
            arm.up();
            deposit.score();
        }else if (!isPressedLB && gamepad1.left_bumper) {
            deposit.toggleOuter();
        } else if (!isPressedRB && gamepad1.right_bumper) {
            deposit.toggleInner();
        }



        intake.update();
        arm.update();
        deposit.update();
        drive.update();
//        launcher.update();
    }

}
