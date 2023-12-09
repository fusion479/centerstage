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

    ElapsedTime armTimer = new ElapsedTime();

    public static int armRaiseDelay = 500;

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
                deposit.intake();
                intake.loop();
                break;
            case READY_BOTTOM:
                lift.bottom();
                arm.up();
                deposit.idle();
                break;
            case READY_LOW:
                lift.low();
                deposit.idle();
                if (armTimer.milliseconds() > armRaiseDelay) {
                    arm.up();
                    up = true;
                    armTimer.reset();
                }
                break;
            case READY_MEDIUM:
                armTimer.reset();
                lift.medium();
                deposit.idle();
                if (armTimer.milliseconds() > armRaiseDelay) {
                    arm.up();
                    up = true;
                    armTimer.reset();
                }
                break;
            case READY_HIGH:
                lift.high();
                deposit.idle();
                if (armTimer.milliseconds() > armRaiseDelay) {
                    arm.up();
                    up = true;
                }
                break;
            case SCORING:
                deposit.score();

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
