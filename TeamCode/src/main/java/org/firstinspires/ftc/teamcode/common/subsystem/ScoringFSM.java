package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ScoringFSM extends Mechanism {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();

    public Lift lift = new Lift();
    public Arm arm = new Arm();
    public Deposit deposit = new Deposit();
    public Intake intake = new Intake();
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
    public static int resetDelay = 400;
    public static int autoIntakeDelay = 2750;

    public boolean isPressedA = false;
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

        if (!isPressedA && gamepad.a) {
            toggleReady();
        }else if (gamepad.b) {
            readyLow();
        } else if (gamepad.x) {
            readyHigh();
        } else if (gamepad.y) {
            readyMedium();
        } else if (gamepad.left_bumper || gamepad.right_bumper) {
            score();
        }

        if (gamepad.right_trigger > 0.1) {
            intake.setPower(gamepad.right_trigger);
        } else if (gamepad.left_trigger > 0.1) {
            intake.setPower(-gamepad.left_trigger);
        } else {
            intake.setPower(0);
        }

        switch (state) {
            case INTAKE:
                // A toggle
                up = false;
                deposit.openOuter();
                deposit.openInner();
                lift.bottom();
                deposit.accepting();
                intake.down();

                if (timer.milliseconds() >= armDelay) {
                    arm.down();
                }
                break;
            case READY_BOTTOM:
                // A toggle
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

                if (timer.milliseconds() > resetDelay && !deposit.innerLocked && !deposit.outerLocked) {
                    readyBottom();
                }
                break;
            case AUTO_INIT:
                intake.down();
                arm.autoInit();
                deposit.idle();

                if (timer.milliseconds() > autoIntakeDelay) {
                    intake.up();
                }
        }

        if (!isPressedRB && gamepad.right_bumper) {
            deposit.toggleOuter();
        }

        if (!isPressedLB && gamepad.left_bumper) {
            deposit.toggleInner();
        }

        isPressedA = gamepad.a;
        isPressedLB = gamepad.left_bumper; // toggle inner pixel
        isPressedRB = gamepad.right_bumper; // toggle outer pixel

        lift.update();
        arm.update();
        deposit.update();
        intake.update();

//        telemetry.addData("lift 0 current", lift.motors[0].getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("lift 1 current", lift.motors[1].getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("intake current", intake.intake.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("LIFT POSITION", lift.getPosition());
//        telemetry.addData("LIFT POWER", Lift.power);
//        telemetry.addData("LIFT ERROR", Lift.error);
//        telemetry.addData("LIFT TARGET", Lift.target);

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
        timer.reset();
    }

    public void autoInit() {
        state = STATES.AUTO_INIT;
        timer.reset();
    }

    public void toggleReady() {
        if (state != STATES.READY_BOTTOM) {
            readyBottom();
        } else {
            intake();
        }
    }
}
