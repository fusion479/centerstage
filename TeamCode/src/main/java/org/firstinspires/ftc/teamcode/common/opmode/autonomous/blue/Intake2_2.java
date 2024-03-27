package org.firstinspires.ftc.teamcode.common.opmode.autonomous.blue;

import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.APRILTAG_TIMEOUT;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.CLOSE_INITIAL;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.CLOSE_LEFT_SPIKE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.CLOSE_PARK;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.CLOSE_RIGHT_SPIKE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.CLOSE_START;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.INITIAL_FORWARD_DIST;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.LEFT_BACKDROP_PRE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.MIDDLE_BACKDROP_PRE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.MIDDLE_SPIKE_DISTANCE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.POST_APRILTAG_FORWARD;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.POST_PRELOAD_WAIT;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.PRELOAD_SCORE_DELAY;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.RIGHT_BACKDROP_PRE;
import static org.firstinspires.ftc.teamcode.common.opmode.autonomous.AutoConstants.STACK_PICKUP_DELAY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.subsystem.Camera;
import org.firstinspires.ftc.teamcode.common.subsystem.ScoringFSM;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Intake 2+2", group = "_Auto")
public class Intake2_2 extends LinearOpMode {
    private static final double VEL_OFFSET = 25;
    private static final double ACCEL_OFFSET = 35;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime loopTime = new ElapsedTime();
    SampleMecanumDrive drive;
    ScoringFSM scoringFSM = new ScoringFSM();
    private STATES autoState = STATES.TO_STACK;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scoringFSM.init(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-34, 11));

        timer.reset();
        scoringFSM.autoInit();


        TrajectorySequence backdropToStack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringFSM.ready();
                })
                .lineToLinearHeading(
                        new Pose2d(-55, 11),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL - VEL_OFFSET, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - ACCEL_OFFSET))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    scoringFSM.stack();
                })
                .waitSeconds(STACK_PICKUP_DELAY)
                .build();

        drive.followTrajectorySequenceAsync(backdropToStack);

        while (!isStarted() && !isStopRequested()) {
                    scoringFSM.update(gamepad1, gamepad2);
                    tele.addData("score timer", scoringFSM.timer.milliseconds());
                    tele.update();
        }

        while (opModeIsActive() && !isStopRequested()) {
            switch (autoState) {
                case TO_STACK:
                    if (!drive.isBusy()) {
                        autoState = STATES.TO_BACKDROP;
                        TrajectorySequence stackToBackdrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(
                                        new Pose2d(-34, 11),
                                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL - VEL_OFFSET, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL - ACCEL_OFFSET))
                                .build();
                        drive.followTrajectorySequenceAsync(stackToBackdrop);
                    }
                    timer.reset();
                case TO_BACKDROP:
                    if (!drive.isBusy()) {
                        timer.reset();
                        break;
                    }
            }

            drive.update();
            loopTime.reset();
            tele.update();

            scoringFSM.update(gamepad1, gamepad2);
        }
    }

    private enum STATES {
        TO_STACK,
        TO_BACKDROP,
    }
}
