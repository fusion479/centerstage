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
        READY,
        BOTTOM,
        LOW,
        MEDIUM,
        HIGH,
        CUSTOM,
        SCORE,
        CLIMB,
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
    public boolean isPressedRB2 = false;
    public boolean isPressedLB2 = false;
    public boolean isPressedDPadUp = false;
    public boolean isPressedDPadDown = false;
//    public double prevRTrigVal = 0;
//    public boolean isRTrigReleased = false;

    @Override
    public void init(HardwareMap hwMap) {
        lift.init(hwMap);
        arm.init(hwMap);
        deposit.init(hwMap);
        intake.init(hwMap);

        state = STATES.INTAKE;
        up = false;
    }

    public void update(Gamepad gamepad, Gamepad gamepad2) {
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
//            intake();
            intake.setPower(gamepad.right_trigger);
        } else if (gamepad.left_trigger > 0.1) {
            intake.setPower(-gamepad.left_trigger);
        }
//        else if (isRTrigReleased) {
//          ready();
//        }
        else {
            intake.setPower(0);
        }

        if (!isPressedDPadUp && gamepad.dpad_up) {
            custom();
            lift.upALittle();
        } else if (!isPressedDPadDown && gamepad.dpad_down) {
            custom();
            lift.downALittle();
        }

        switch (state) {
            case INTAKE:
                // A toggle
                up = false;
                lift.isClimb = false;
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
                lift.isClimb = false;
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
                lift.isClimb = false;
                deposit.lockInner();
                deposit.lockOuter();
                lift.bottom();
                arm.up();
                deposit.score();
                intake.idle();
                break;
            case LOW:
                up = true;
                lift.isClimb = false;
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
                lift.isClimb = false;
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
                lift.isClimb = false;
                deposit.lockInner();
                deposit.lockOuter();
                lift.high();
                arm.up();
                deposit.score();
                intake.idle();
                break;
            case CUSTOM:
                up = true;
                lift.isClimb = false;
                deposit.lockInner();
                deposit.lockOuter();
                arm.up();
                deposit.score();
                intake.idle();
            case SCORE:
                // Left or Right
                up = true;
                lift.isClimb = false;
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
            case CLIMB:
                up = true;
                lift.isClimb = true;
                arm.climb();
                deposit.idle();
                intake.up();
                if (gamepad2.right_trigger > .1) {
                    lift.setPower(gamepad2.right_trigger);
                } else if (gamepad2.left_trigger > .1) {
                    lift.setPower(-gamepad2.left_trigger);
                } else {
                    lift.setPower(0);
                };
            case AUTO_INIT:
                lift.isClimb = false;
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
        isPressedLB2 = gamepad.left_bumper;
        isPressedRB2 = gamepad.right_bumper;
        isPressedDPadUp = gamepad.dpad_up;
        isPressedDPadDown = gamepad.dpad_down;
        isPressedA = gamepad.a;

//        if (prevRTrigVal > 0 && gamepad.right_trigger == 0) {
//            isRTrigReleased = true;
//        } else {
//            isRTrigReleased = false;
//        }
//        prevRTrigVal = gamepad.right_trigger;

        lift.update();
        arm.update();
        deposit.update();
        intake.update();

//        telemetry.addData("is pressed LB", isPressedLB);
//        telemetry.addData("is pressed RB", isPressedRB);
//        telemetry.addData("looptime", loopTimeTimer.milliseconds());
//        loopTimeTimer.reset();

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

    public void custom() {
        state = STATES.CUSTOM;
    }

    public void score() {
        state = STATES.SCORE;
        timer.reset();
    }

    public void climb() {
        state = STATES.CLIMB;
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
