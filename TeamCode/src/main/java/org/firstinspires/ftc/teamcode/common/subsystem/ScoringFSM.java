package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class ScoringFSM extends Mechanism {
    Lift lift = new Lift();
    Arm arm = new Arm();
    Deposit deposit = new Deposit();
    Intake intake = new Intake();
    public enum STATES {
        INTAKE,
        READY_BOTTOM,
        LOW,
        MEDIUM,
        HIGH,
    };

    public STATES state;
    public boolean up;

    ElapsedTime armTimer = new ElapsedTime();

    public static int armRaiseDelay = 500;

    @Override
    public void init(HardwareMap hwMap) {
        lift.init(hwMap);
        arm.init(hwMap);
        deposit.init(hwMap);
        intake.init(hwMap);
        state = STATES.INTAKE;
        up = false;
    }

    public void update() {
        switch (state) {
            case INTAKE:
                lift.bottom();
                arm.down();
                deposit.accepting();
                break;
            case READY_BOTTOM:
                lift.bottom();
                arm.up();
                deposit.ready();
                break;
            case LOW:
                lift.low();
                arm.up();
                deposit.ready();
                break;
            case MEDIUM:
                lift.medium();
                arm.up();
                deposit.ready();
                break;
            case HIGH:
                lift.high();
                arm.up();
                deposit.ready();
                break;
        }
    }

    public void intake() {
        state = STATES.INTAKE;
    }

    public void readyLow() {
        state = STATES.LOW;
    }

    public void readyBottom() {
        state = STATES.READY_BOTTOM;
    }

    public void readyHigh() {
        state = STATES.HIGH;
    }

    public void readyMedium() {
        state = STATES.MEDIUM;
    }
}
