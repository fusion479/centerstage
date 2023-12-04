package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class ScoringFSM extends Mechanism {
    Lift lift = new Lift();
    Arm arm = new Arm();
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
    public boolean up;

    @Override
    public void init(HardwareMap hwMap) {
        lift.init(hwMap);
        arm.init(hwMap);
        deposit.init(hwMap);
        intake.init(hwMap);
        state = STATES.INTAKING;
        up = false;
    }

    public void loop() {
        switch (state) {
            case INTAKING:
                if (up) {
                    lift.bottom();
                    up =  false;
                }
                arm.down();
                deposit.setIntakePos();
                intake.loop();
                break;
            case READY_BOTTOM:
                lift.bottom();
                arm.up();
                break;
            case READY_LOW:
                lift.low();
                arm.up();
                up = true;
                break;
            case READY_MEDIUM:
                lift.medium();
                arm.up();
                up = true;
                break;
            case READY_HIGH:
                lift.high();
                arm.up();
                up = true;
                break;
            case SCORING:
                deposit.setScorePos();
                break;
        }
    }

    public void intake() {
        state = STATES.INTAKING;
    }

    public void readyLow() {
        state = STATES.READY_LOW;
    }

    public void readyBottom() {
        state = STATES.READY_BOTTOM;
    }

    public void readyHigh() {
        state = STATES.READY_HIGH;
    }

    public void readyMedium() {
        state = STATES.READY_MEDIUM;
    }

    public void score() {
        state = STATES.SCORING;
    }
}
