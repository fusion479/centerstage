package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Robot extends Mechanism {
//    private final ScoringFSM score = new ScoringFSM();
    private MecanumDrive drive;
    private Pose2d pos = new Pose2d(0, 0, 0);

//    private boolean isPressedx = false;

    @Override
    public void init(HardwareMap hwMap) {
        drive = new MecanumDrive(hwMap, pos);
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(-gamepad1.right_stick_x, -gamepad1.left_stick_y),
                        -gamepad1.left_stick_y
                )
        );
        drive.updatePoseEstimate();
//        if (!isPressedx && gamepad1.x) {
//           score.climber.toggle();
//        }
//        isPressedx = gamepad1.x;
//        score.loop();
    }

}
