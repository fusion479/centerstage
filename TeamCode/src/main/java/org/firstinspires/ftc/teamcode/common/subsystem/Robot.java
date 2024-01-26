package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Robot extends Mechanism {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();

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
    public boolean isPressedDPadDown = false;
    public boolean isPressedDPadUp = false;

    @Override
    public void init(HardwareMap hwMap) {
        drive = new SampleMecanumDrive(hwMap);
        lift.init(hwMap);
        arm.init(hwMap);
        deposit.init(hwMap);
        intake.init(hwMap);

        arm.up();
        deposit.accepting();
        intake.idle();
        deposit.openOuter();
        deposit.openInner();
        lift.bottom();
    }

    public void update(Gamepad gamepad, Gamepad gamepad2) {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad.left_stick_y,
                        -gamepad.left_stick_x,
                        -gamepad.right_stick_x
                )
        );

        if (gamepad.a) {
            lift.bottom();
            arm.down();
            deposit.accepting();
        } else if (gamepad.dpad_down) {
            intake.intaking();
        } else if (gamepad.dpad_up) {
            intake.idle();
        } else if (gamepad.b) {
            arm.up();
            deposit.ready();
        } else if (gamepad.x) {
            lift.medium();
            arm.up();
            deposit.score();
        } else if (gamepad.y) {
            lift.high();
            arm.up();
            deposit.score();
        } else if (gamepad.dpad_left) {
            deposit.openOuter();
            deposit.openInner();
        } else if (gamepad.dpad_right) {
            deposit.lockOuter();
            deposit.lockInner();
        }

        if (gamepad.right_trigger > 0) {
            intake.setPower(gamepad.right_trigger);
        } else if (gamepad.left_trigger > 0) {
            intake.setPower(-gamepad.left_trigger);
        } else {
            intake.setPower(0);
        }

        isPressedX = gamepad.x; // high
        isPressedY = gamepad.y;  // medium
        isPressedA = gamepad.a; // ready bottom
        isPressedB = gamepad.b; // low
        isPressedLB = gamepad.left_bumper; // toggle inner pixel
        isPressedRB = gamepad.right_bumper; // toggle outer pixel
        isPressedDPadUp = gamepad.dpad_up;
        isPressedDPadDown = gamepad.dpad_down;

        lift.update();
        intake.update();
        arm.update();
        deposit.update();
        drive.update();

        telemetry.addData("Outer Locked?", Deposit.outerLocked);
        telemetry.addData("Inner Locked?", Deposit.innerLocked);
        telemetry.update();
    }

}
