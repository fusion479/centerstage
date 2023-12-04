package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Robot extends Mechanism {
//    private final ScoringFSM score = new ScoringFSM();
    private SampleMecanumDrive drive;
//    private boolean isPressedx = false;

    @Override
    public void init(HardwareMap hwMap) {
        drive = new SampleMecanumDrive(hwMap);
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
//        if (!isPressedx && gamepad1.x) {
//           score.climber.toggle();
//        }
//        isPressedx = gamepad1.x;
//        score.loop();
    }

}
