package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Robot extends Mechanism {
    private ScoringFSM score = new ScoringFSM();
    private MecanumDrive drive;
    private Pose2d pos = new Pose2d(0, 0, 0);

    private boolean isPressedx = false;

    @Override
    public void init(HardwareMap hwMap) {
        drive = new MecanumDrive(hwMap, pos);
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        if (!isPressedx && gamepad1.x) {
           score.climber.toggle();
        }
        isPressedx = gamepad1.x;
        score.loop();
    }

}
