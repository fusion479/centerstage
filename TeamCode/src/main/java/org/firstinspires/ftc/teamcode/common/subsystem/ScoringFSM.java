package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
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
        SCORE
    };

    public STATES state;
    public boolean up;

    public ElapsedTime armTimer = new ElapsedTime();
    public static int armDelay = 300;

    public boolean isPressedRB = false;
    public boolean isPressedLB = false;

    @Override
    public void init(HardwareMap hwMap) {
        lift.init(hwMap);
        arm.init(hwMap);
        deposit.init(hwMap);
        intake.init(hwMap);

        state = STATES.INTAKE;
        up = false;
    }

    public void update(Gamepad gamepad) {
        switch (state) {
            case INTAKE:
                // A
                deposit.openOuter();
                deposit.openInner();
                lift.bottom();
                deposit.accepting();
                intake.intaking();

                if (armTimer.milliseconds() >= armDelay) {
                    arm.down();
                }
                break;
            case READY_BOTTOM:
                // B
                deposit.lockInner();
                deposit.lockOuter();

                if (armTimer.milliseconds() >= 100) {
                    lift.bottom();
                    arm.ready();
                }

                if (armTimer.milliseconds() >= 500) {
                    deposit.ready();
                    intake.idle();
                }

                break;
            case LOW:
                deposit.lockInner();
                deposit.lockOuter();
                lift.low();
                arm.up();
                deposit.score();
                intake.idle();
                break;
            case MEDIUM:
                // Y
                deposit.lockInner();
                deposit.lockOuter();
                lift.medium();
                arm.up();
                deposit.score();
                intake.idle();
                break;
            case HIGH:
                // X
                deposit.lockInner();
                deposit.lockOuter();
                lift.high();
                arm.up();
                deposit.score();
                intake.idle();
                break;
            case SCORE:
                // Left or Right
                arm.up();
                deposit.score();
                intake.idle();
                break;
        }

        if (gamepad.right_trigger > 0.1) {
            intake.setPower(gamepad.right_trigger);
        } else if (gamepad.left_trigger > 0.1) {
            intake.setPower(-gamepad.left_trigger);
        } else {
            intake.setPower(0);
        }

        if (!isPressedRB && gamepad.right_bumper) {
            deposit.toggleOuter();
        }

        if (!isPressedLB && gamepad.left_bumper) {
            deposit.toggleInner();
        }

        isPressedLB = gamepad.left_bumper; // toggle inner pixel
        isPressedRB = gamepad.right_bumper; // toggle outer pixel

        lift.update();
        arm.update();
        deposit.update();
        intake.update();
    }

    public void intake() {
        armTimer.reset();
        state = STATES.INTAKE;
    }

    public void readyLow() {
        state = STATES.LOW;
    }

    public void readyBottom() {
        armTimer.reset();
        state = STATES.READY_BOTTOM;
    }

    public void readyHigh() {
        state = STATES.HIGH;
    }

    public void readyMedium() {
        state = STATES.MEDIUM;
    }

    public void score() {
        state = STATES.SCORE;
    }
}
