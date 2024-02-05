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
//    public RevBlinkinLedDriver lights = new RevBlinkinLedDriver();
    public enum STATES {
        INTAKE,
        READY,
        BOTTOM,
        LOW,
        MEDIUM,
        HIGH,
        SCORE,
        CLIMB_UP,
        CLIMB_DOWN,
        AUTO_INIT
    };

    public STATES state;
    public boolean up;
    public boolean isAuto;

    public ElapsedTime timer = new ElapsedTime();
    public ElapsedTime loopTimeTimer = new ElapsedTime();
    public static int armDelay = 300;
    public static int resetDelay = 400;
    public static int autoIntakeDelay = 1000;

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
            low();
        } else if (gamepad.x) {
            high();
        } else if (gamepad.y) {
            medium();
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
            case READY:
                // A toggle
                up = false;
                deposit.lockInner();
                deposit.lockOuter();

                if (timer.milliseconds() >= 25) {
                    lift.bottom();
                    arm.ready();
                }

                if (timer.milliseconds() >= 125) {
                    deposit.ready();
                }

                if (timer.milliseconds() >= 600) {
                    intake.idle();
                }

                break;
            case BOTTOM:
                up = true;
                deposit.lockInner();
                deposit.lockOuter();
                lift.bottom();
                arm.up();
                deposit.score();
                intake.idle();
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
                    if (!isAuto) {
                        ready();
                    } else {
                        deposit.openOuter();
                        deposit.openInner();
                    }
                }
                break;
            case CLIMB_UP:
                up = true;
                arm.climb();
                deposit.idle();
                lift.high();
                intake.up();
            case CLIMB_DOWN:
                up = false;
                arm.climb();
                deposit.idle();
                lift.climb();
                intake.up();
            case AUTO_INIT:
                intake.down();
                arm.autoInit();
                deposit.ready();

                if (timer.milliseconds() > autoIntakeDelay) {
                    intake.up();
                    deposit.lockInner();
                    deposit.lockOuter();
                }

        }

        if (!isPressedRB && gamepad.right_bumper) {
            deposit.toggleOuter();
        } else if (!isPressedLB && gamepad.left_bumper) {
            deposit.toggleInner();
        }

        isPressedLB = gamepad.left_bumper; // toggle inner pixel
        isPressedRB = gamepad.right_bumper; // toggle outer pixel
        isPressedA = gamepad.a;


        lift.update();
        arm.update();
        deposit.update();
        intake.update();

        telemetry.addData("is pressed LB", isPressedLB);
        telemetry.addData("is pressed RB", isPressedRB);
        telemetry.addData("looptime", loopTimeTimer.milliseconds());
        loopTimeTimer.reset();
        telemetry.update();

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

    public void ready() {
        timer.reset();
        state = STATES.READY;
    }

    public void bottom() {
        state = STATES.BOTTOM;
    }

    public void low() {
        state = STATES.LOW;
    }

    public void medium() {
        state = STATES.MEDIUM;
    }

    public void high() {
        state = STATES.HIGH;
    }

    public void score() {
        state = STATES.SCORE;
        timer.reset();
    }

    public void climbUp() {
        state = STATES.CLIMB_UP;
    }

    public void climbDown() {
        state = STATES.CLIMB_DOWN;
    }

    public void autoInit() {
        state = STATES.AUTO_INIT;
        timer.reset();
    }

    public void toggleReady() {
        if (state != STATES.READY) {
            ready();
        } else {
            intake();
        }
    }
}
