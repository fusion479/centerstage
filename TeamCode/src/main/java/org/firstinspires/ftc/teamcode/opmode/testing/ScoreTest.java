package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.subsystem.ScoringFSM;

@TeleOp(name = "ScoreFSM Test", group = "testing")
@Config
public class ScoreTest extends LinearOpMode {
    ScoringFSM scoringFSM = new ScoringFSM();

    @Override
    public void runOpMode() throws InterruptedException {
        scoringFSM.init(hardwareMap);

        waitForStart();

        // Scan servo till stop pressed.
        while(opModeIsActive()){
            scoringFSM.loop();
        }
    }
}
