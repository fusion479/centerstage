package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
public class DemoDrivetrain extends Subsystem {
    public final MecanumDrive drive;

    public DemoDrivetrain(final HardwareMap hwMap, final MultipleTelemetry telemetry) {
        super(telemetry);

        this.drive = new MecanumDrive(hwMap, new Pose2d(0, 0, 0));
        // this.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void manualDrive(final GamepadEx gamepad) {
        this.drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        0.3 * gamepad.getLeftY(),
                        -0.3 * gamepad.getLeftX()
                ),
                -0.3 * gamepad.getRightX()
        ));
        this.drive.updatePoseEstimate();
    }
}