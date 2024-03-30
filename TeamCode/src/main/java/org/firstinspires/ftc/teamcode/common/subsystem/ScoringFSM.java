package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class ScoringFSM extends Mechanism {
    public static int armDelay = 300;
    public static int resetDelay = 150;
    public static int autoIntakeDelay = 100;
    public static int colorSensorDelay = 50;
    public static int readyDelay = 75;

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
    public boolean wasGigaHigh;

    public ElapsedTime timer = new ElapsedTime();
    public ElapsedTime sensorTimer = new ElapsedTime();
    public ElapsedTime loopTimeTimer = new ElapsedTime();
    public boolean isPressedA = false;
    public boolean isPressedA2 = false;
    public boolean isPressedRB = false;
    public boolean isPressedLB = false;
    public boolean isPressedRB2 = false;
    public boolean isPressedLB2 = false;
    public boolean isPressedDPadUp = false;
    public boolean isPressedDPadDown = false;
    public boolean isPressedDPadRight = false;
    public boolean isPressedDPadLeft = false;
    public boolean isPressedDPadUp2 = false;
    public boolean isPressedDPadDown2 = false;
    public boolean isPressedDPadRight2 = false;

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
        if ((!isPressedA && gamepad.a) || (!isPressedA2 && gamepad2.a)) {
            toggleReady();
        } else if (gamepad.b || gamepad2.b) {
            low();
        } else if (gamepad.x || gamepad2.x) {
            high();
        } else if (gamepad.y || gamepad2.y) {
            medium();
        } else if (gamepad.left_bumper || gamepad.right_bumper) {
            score();
        }

        if (gamepad.right_stick_button) {
            intake();
        }

        if (gamepad.right_trigger > 0.1) {
            intake.setPower(gamepad.right_trigger * .8);
        } else if (gamepad.left_trigger > 0.1) {
            intake.setPower(-gamepad.left_trigger * .8);
        } else {
            intake.setPower(0);
        }

        if (!isPressedDPadUp && gamepad.dpad_up) {
            custom();
            lift.upALittle();
        } else if (!isPressedDPadDown && gamepad.dpad_down) {
            custom();
            lift.downALittle();
        }  else if (!isPressedDPadRight && gamepad.dpad_right) {
            bottomLow();
        } else if (!isPressedDPadLeft && gamepad.dpad_left) {
            gigaHigh();
        }

        if (!isPressedDPadUp2 && gamepad2.dpad_up) {
            custom();
            lift.upALittle();
        } else if (!isPressedDPadDown2 && gamepad2.dpad_down) {
            custom();
            lift.downALittle();
        }  else if (!isPressedDPadRight2 && gamepad2.dpad_right) {
            bottomLow();
        }

        switch (state) {
            case INTAKE:
                hasPixel();


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
                wasGigaHigh = false;
                deposit.lockInner();
                deposit.lockOuter();
//                intake.setPower(0);

                if (isAuto) {
                    intake.idle();

                    if (timer.milliseconds() >= 75) {
                        lift.bottom();
                        arm.ready();
                        deposit.ready();
                    }
                } else {
                    if (timer.milliseconds() >= 100) {
                        lift.bottom();
                        arm.ready();
                    }

                    if (timer.milliseconds() >= 150) {
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
                }

                if (timer.milliseconds() >= readyDelay) {
                    arm.score();
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
                }

                if (timer.milliseconds() >= readyDelay) {
                    arm.score();
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
                }

                if (timer.milliseconds() >= readyDelay) {
                    arm.score();
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
                }

                if (timer.milliseconds() >= readyDelay) {
                    arm.score();
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
                }

                if (timer.milliseconds() >= readyDelay) {
                    arm.score();
                    deposit.score();
                }

                if (timer.milliseconds() >= 150) {
                    intake.idle();
                }
                break;
            case GIGA_HIGH:
                up = true;
                lift.isClimb = false;
                wasGigaHigh = true;
                deposit.lockInner();
                deposit.lockOuter();
                if (timer.milliseconds() >= 25) {
                    lift.max();
                }

                if (timer.milliseconds() >= readyDelay) {
                    arm.maxScore();
                    deposit.maxScore();
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

                if (timer.milliseconds() >= readyDelay) {
                    arm.score();
                    deposit.score();
                }

                if (timer.milliseconds() >= 150) {
                    intake.idle();
                }
                break;
            case SCORE:
                // Left or Right
                up = true;
                lift.isClimb = false;
                if (!wasGigaHigh) {
                    arm.score();
                    deposit.score();
                    intake.idle();
                }

                if (timer.milliseconds() > resetDelay && !deposit.innerLocked && !deposit.outerLocked) {
                    if (!isAuto) {
                        if (resetCounter < 3) {
                            if (!wasGigaHigh) {
                                lift.upALittle();
                            }
                        }
                        resetCounter++;

                        if (timer.milliseconds() > resetDelay + 200) {
                            ready();
                        }
                    } else {
                        if (resetCounter < 3) {
                            // TODO: ??
                        }
                        resetCounter++;

                        deposit.openOuter();
                        deposit.openInner();
                    }
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
                lift.bottom();
                deposit.openInner();
                deposit.openOuter();

                if (timer.milliseconds() >= 25) {
                    intake.stack();
                }


                if (timer.milliseconds() >= 125) {
                    lift.upALittle();
                    arm.stack();
                    deposit.stack();
                }

                if (timer.milliseconds() >= 400) {
                    lift.downALittle();
                }

                if (timer.milliseconds() >= 750) {
                    intake.setPower(1);
                }

                hasPixel();
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
        isPressedDPadRight = gamepad.dpad_right;
        isPressedDPadLeft = gamepad.dpad_left;
        isPressedDPadUp2 = gamepad2.dpad_up;
        isPressedDPadDown2 = gamepad2.dpad_down;
        isPressedDPadRight2 = gamepad2.dpad_right;
        isPressedA = gamepad.a;
        isPressedA2 = gamepad2.a;

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
        telemetry.addData("outer reading", pixelSensor.outerSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("inner reading", pixelSensor.innerSensor.getDistance(DistanceUnit.MM));
        telemetry.update();
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

    public void gigaHigh() {
        timer.reset();
        state = STATES.GIGA_HIGH;
    }

    public void hasPixel() {
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
        AUTO_INIT,
        STACK,
        BOTTOM_LOW,
        GIGA_HIGH
    }
}
