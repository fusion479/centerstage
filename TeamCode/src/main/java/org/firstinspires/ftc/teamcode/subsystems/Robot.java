package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot extends Mechanism {
    ScoringFSM scoringFSM = new ScoringFSM();
    Launcher launcher = new Launcher();
    Climber climber = new Climber();

    @Override
    public void init(HardwareMap hwMap) {
        scoringFSM.init(hwMap);
        launcher.init(hwMap);
        climber.init(hwMap);
    }

    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        if

    }
}
