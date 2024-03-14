package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ScoringFSM extends Mechanism {
    public static int armDelay = 300;
    public static int resetDelay = 75;
    public static int autoIntakeDelay = 100;
    public static int colorSensorDelay = 75;

    public Lift lift = new Lift();
    public Arm arm = new Arm();
    public Deposit deposit = new Deposit();
    public Intake intake = new Intake();
    public PixelSensor pixelSensor = new PixelSensor();

    public int resetCounter = 0;
    public int sensorCounter = 0;

    public STATES state;
    public boolean up;
    public boolean isAuto;

    public ElapsedTime timer = new ElapsedTime();
    public ElapsedTime sensorTimer = new ElapsedTime();
    public ElapsedTime loopTimeTimer = new ElapsedTime();
    public boolean isPressedA = false;
    public boolean isPressedRB = false;
    public boolean isPressedLB = false;
    public boolean isPressedRB2 = false;
    public boolean isPressedLB2 = false;
    public boolean isPressedDPadUp = false;
    public boolean isPressedDPadDown = false;
    public boolean isPressedDPadRight = false;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();

    @Override
    public void init(HardwareMap hwMap) {
        lift.init(hwMap);
        arm.init(hwMap);
        deposit.init(hwMap);
        intake.init(hwMap);
        pixelSensor.init(hwMap);

        state = STATES.INTAKE;
        up = false;
    }

    public void update(Gamepad gamepad, Gamepad gamepad2) {
        if (!isPressedA && gamepad.a) {
            toggleReady();
        } else if (gamepad.b) {
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
        } else if (!isPressedDPadRight && gamepad.dpad_right) {
            stack();
        }

        switch (state) {
            case INTAKE:
                if (pixelSensor.hasPixel()) {
                    if (sensorCounter == 0) {
                        sensorTimer.reset();
                        sensorCounter++;
                    } else if (sensorTimer.milliseconds() >= colorSensorDelay) {
                        state = STATES.READY;
                    }
                } else {
                    sensorCounter = 0;
                }


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

                if (isAuto) {
                    intake.idle();

                    if (timer.milliseconds() >= 75) {
                        lift.bottom();
                        arm.ready();
                        deposit.ready();
                    }
                } else {
                    if (timer.milliseconds() >= 25) {
                        lift.bottom();
                        arm.ready();
                    }

                    if (timer.milliseconds() >= 125) {
                        deposit.ready();
                    }


                    if (timer.milliseconds() >= 300) {
                        intake.idle();
                    }
                }

                break;
            case BOTTOM:
                up = true;
                lift.isClimb = false;
                deposit.lockInner();
                deposit.lockOuter();

                if (timer.milliseconds() >= 25) {
                    lift.bottom();
                    arm.ready();
                }

                if (timer.milliseconds() >= 125) {
                    arm.up();
                    deposit.score();
                }

                if (timer.milliseconds() >= 150) {
                    intake.idle();
                }
                break;
            case BOTTOM_LOW:
                up = true;
                lift.isClimb = false;
                deposit.lockInner();
                deposit.lockOuter();


                if (timer.milliseconds() >= 25) {
                    lift.bottomLow();
                    arm.ready();
                }

                if (timer.milliseconds() >= 125) {
                    arm.up();
                    deposit.score();
                }

                if (timer.milliseconds() >= 150) {
                    intake.idle();
                }
                break;
            case LOW:
                up = true;
                lift.isClimb = false;
                deposit.lockInner();
                deposit.lockOuter();
                if (timer.milliseconds() >= 25) {
                    lift.low();
                    arm.ready();
                }

                if (timer.milliseconds() >= 125) {
                    arm.up();
                    deposit.score();
                }

                if (timer.milliseconds() >= 150) {
                    intake.idle();
                }
                break;
            case MEDIUM:
                // Y
                up = true;
                lift.isClimb = false;
                deposit.lockInner();
                deposit.lockOuter();
                if (timer.milliseconds() >= 25) {
                    lift.medium();
                    arm.ready();
                }

                if (timer.milliseconds() >= 125) {
                    arm.up();
                    deposit.score();
                }

                if (timer.milliseconds() >= 150) {
                    intake.idle();
                }
                break;
            case HIGH:
                // X
                up = true;
                lift.isClimb = false;
                deposit.lockInner();
                deposit.lockOuter();
                if (timer.milliseconds() >= 25) {
                    lift.high();
                    arm.ready();
                }

                if (timer.milliseconds() >= 125) {
                    arm.up();
                    deposit.score();
                }

                if (timer.milliseconds() >= 150) {
                    intake.idle();
                }
                break;
            case CUSTOM:
                up = true;
                lift.isClimb = false;
                deposit.lockInner();
                deposit.lockOuter();
                if (timer.milliseconds() >= 90) {
                    arm.ready();
                }

                if (timer.milliseconds() >= 125) {
                    arm.up();
                    deposit.score();
                }

                if (timer.milliseconds() >= 150) {
                    intake.idle();
                }
            case SCORE:
                // Left or Right
                up = true;
                lift.isClimb = false;
                arm.up();
                deposit.score();
                intake.idle();

                if (timer.milliseconds() > resetDelay && !deposit.innerLocked && !deposit.outerLocked) {
                    if (!isAuto) {
                        if (resetCounter < 3) {
                            lift.upALittle();
                        }
                        resetCounter++;

                        if (timer.milliseconds() > resetDelay + 250) {
                            ready();
                        }
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
                }
                break;
            case AUTO_INIT:
                lift.isClimb = false;
                isAuto = true;
                intake.down();
                arm.autoInit();
                deposit.autoInit();

                if (timer.milliseconds() > autoIntakeDelay) {
                    intake.up();
                    deposit.lockInner();
                    deposit.lockOuter();
                }
                break;
            case STACK:
                up = true;
                lift.isClimb = false;

                intake.stack();
                arm.down();
                lift.stack(); // 75
                deposit.autoStack();
                deposit.openInner();
                deposit.openOuter();

                if (timer.milliseconds() >= 25) {
                    intake.stack();
                }


                if (timer.milliseconds() >= 125) {
                    deposit.autoStack();
                }


                if (pixelSensor.hasPixel()) {
                    if (sensorCounter == 0) {
                        sensorTimer.reset();
                        sensorCounter++;
                        intake.setPower(1);
                    } else if (sensorTimer.milliseconds() >= 200) {
                        state = STATES.READY;
                    }
                } else {
                    sensorCounter = 0;
                }
                break;
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
        timer.reset();
        state = STATES.BOTTOM;
    }

    public void low() {
        timer.reset();
        state = STATES.LOW;
    }

    public void bottomLow() {
        timer.reset();
        state = STATES.BOTTOM_LOW;
    }

    public void medium() {
        timer.reset();
        state = STATES.MEDIUM;
    }

    public void high() {
        timer.reset();
        state = STATES.HIGH;
    }

    public void custom() {
        timer.reset();
        state = STATES.CUSTOM;
    }

    public void score() {
        timer.reset();
        state = STATES.SCORE;
        resetCounter = 0;
    }

    public void climb() {
        state = STATES.CLIMB;
    }

    public void autoInit() {
        timer.reset();
        state = STATES.AUTO_INIT;
    }

    public void stack() {
        timer.reset();
        state = STATES.STACK;
    }

    public void toggleReady() {
        if (state != STATES.READY) {
            ready();
        } else {
            intake();
        }
    }

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
        AUTO_INIT,
        STACK,
        BOTTOM_LOW,
    }
}
