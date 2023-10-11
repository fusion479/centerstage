package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class ScoringFSM {
    Lift lift;
    Arm arm;
    Deposit deposit;

    public enum STATES {
        INTAKING,
        READY_BOTTOM,
        READY_LOW,
        READY_MEDIUM,
        READY_HIGH,
        SCORING,
    };

    public STATES state;

    public ScoringFSM(HardwareMap hwMap) {
        lift = new Lift(hwMap);
        arm = new Arm(hwMap);
        deposit = new Deposit(hwMap);

        state = STATES.INTAKING;
    }

    public void loop() {
        switch (state) {
            case INTAKING:
                arm.down();
                deposit.open();
            case READY_BOTTOM:
                arm.up();
            case READY_LOW:
            case READY_MEDIUM:
            case READY_HIGH:
            case SCORING:
        }
    }
}
