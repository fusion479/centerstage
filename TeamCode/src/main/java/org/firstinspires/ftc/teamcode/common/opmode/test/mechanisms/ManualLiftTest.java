package org.firstinspires.ftc.teamcode.common.opmode.test.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.subsystem.Intake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Manual Lift Test", group = "__")
@Config
public class ManualLiftTest extends LinearOpMode {
    SampleMecanumDrive drive;
    private final DcMotorEx[] motors = new DcMotorEx[2];
    Intake intake = new Intake();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        motors[0] = hardwareMap.get(DcMotorEx.class, "liftLeft");
        motors[1] = hardwareMap.get(DcMotorEx.class, "liftRight");

        motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[0].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);

        intake.init(hardwareMap);
        intake.up();
        intake.update();

        waitForStart();
        while(opModeIsActive()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            if (gamepad1.right_trigger > .1) {
                motors[0].setPower(gamepad1.right_trigger);
                motors[1].setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > .1) {
                motors[0].setPower(-gamepad1.left_trigger);
                motors[1].setPower(-gamepad1.left_trigger);
            } else {
                motors[0].setPower(0);
                motors[1].setPower(0);
            }
            intake.up();

            intake.update();
            multipleTelemetry.addData("current position: ", motors[0].getCurrentPosition());
            multipleTelemetry.update();
        }
    }
}
