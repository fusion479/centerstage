package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;

public class ScoringFSM extends Mechanism {
    Lift lift = new Lift();
    Arm arm = new Arm();
    Climber climber = new Climber();
    Deposit deposit = new Deposit();
    Intake intake;
    public enum STATES {
        INTAKING,
        READY_BOTTOM,
        READY_LOW,
        READY_MEDIUM,
        READY_HIGH,
        SCORING,
        LAUNCH,
    };

    public STATES state;

    @Override
    public void init(HardwareMap hwMap) {
        lift.init(hwMap);
        arm.init(hwMap);
        climber.init(hwMap);
        deposit.init(hwMap);
        state = STATES.INTAKING;
    }

    public void loop() {
        switch (state) {
            case INTAKING:
                arm.down();
                deposit.setIntakePos();
            case READY_BOTTOM:
                arm.up();
            case READY_LOW:
            case READY_MEDIUM:
            case READY_HIGH:
            case SCORING:
        }
    }
}
