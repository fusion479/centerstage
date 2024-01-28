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
        SCORE,
        AUTO_INIT
    };

    public STATES state;
    public boolean up;

    public ElapsedTime timer = new ElapsedTime();
    public static int armDelay = 300;
    public static int autoIntakeDelay = 2750;

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
                up = false;
                deposit.openOuter();
                deposit.openInner();
                lift.bottom();
                deposit.accepting();
                intake.intaking();

                if (timer.milliseconds() >= armDelay) {
                    arm.down();
                }
                break;
            case READY_BOTTOM:
                // B
                up = false;
                deposit.lockInner();
                deposit.lockOuter();

                if (timer.milliseconds() >= 100) {
                    lift.bottom();
                    arm.ready();
                }

                if (timer.milliseconds() >= 500) {
                    deposit.ready();
                    intake.idle();
                }

                break;
            case LOW:
                up = true;
                deposit.lockInner();
                deposit.lockOuter();
                lift.low();
                arm.up();
                deposit.score();
                intake.idle();
                break;
            case MEDIUM:
                // Y
                up = true;
                deposit.lockInner();
                deposit.lockOuter();
                lift.medium();
                arm.up();
                deposit.score();
                intake.idle();
                break;
            case HIGH:
                // X
                up = true;
                deposit.lockInner();
                deposit.lockOuter();
                lift.high();
                arm.up();
                deposit.score();
                intake.idle();
                break;
            case SCORE:
                // Left or Right
                up = true;
                arm.up();
                deposit.score();
                intake.idle();
                break;
            case AUTO_INIT:
                intake.intaking();
                arm.autoInit();
                deposit.idle();

                if (timer.milliseconds() > autoIntakeDelay) {
                    intake.up();
                }
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
        timer.reset();
        state = STATES.INTAKE;
    }

    public void readyLow() {
        state = STATES.LOW;
    }

    public void readyBottom() {
        timer.reset();
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

    public void autoInit() {
        state = STATES.AUTO_INIT;
        timer.reset();
    }
}
