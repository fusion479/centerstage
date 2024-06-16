package org.firstinspires.ftc.teamcode.opmodes.auton.blue.camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Subsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Config
public class Camera extends Subsystem {
    public final static double SPEED_GAIN = 0.03;
    public final static double STRAFE_GAIN = 0.025;
    public final static double TURN_GAIN = 0.03;
    public final static double MAX_AUTO_SPEED = 0.3;
    public final static double MAX_AUTO_STRAFE = 0.3;
    public final static double DESIRED_DISTANCE = 5;
    public static double MAX_AUTO_TURN = 0.15;

    public static int desiredTagId = 2;
    private final Pipeline pipeline;
    private final Color color;
    private OpenCvCamera camera;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private AprilTagDetection desiredTag;
    private boolean isFinished = false;

    public Camera(final Color color, final MultipleTelemetry telemetry) {
        super(telemetry);

        this.pipeline = new Pipeline(color);
        this.color = color;
    }


    public void initCamera(HardwareMap hwMap) {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        this.camera = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        this.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 0);
                camera.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {
                Camera.super.getTelemetry().addLine("Error occured with camera initialization.");
            }
        });
    }

    public void initVisionPortal(HardwareMap hwMap, int region) {
        desiredTagId = this.color == Color.BLUE ? region : region + 3;
        this.aprilTagProcessor = new AprilTagProcessor.Builder().build();
        this.aprilTagProcessor.setDecimation(3);
        this.visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "camera"))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    public void moveRobot(MecanumDrive drivetrain) {
        double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
        double headingError = desiredTag.ftcPose.bearing;
        double yawError = desiredTag.ftcPose.yaw;

        double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
        double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        super.getTelemetry().addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        super.getTelemetry().addData("Errors", "Range %5.2f, YAW %5.2f, Heading %5.2f", rangeError, headingError, yawError);

        double leftFrontPower = drive - strafe - turn;
        double rightFrontPower = drive + strafe + turn;
        double leftBackPower = drive + strafe - turn;
        double rightBackPower = drive - strafe + turn;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        if (Math.abs(headingError) < 5) {
            MAX_AUTO_TURN = 0.1;
        } else if (Math.abs(headingError) < 2.5) {
            MAX_AUTO_TURN = 0.05;
        }


        if (((Math.abs(rangeError) + Math.abs(yawError)) / 2) > 0.12 && Math.abs(headingError) > .04) {
            drivetrain.setMotorPowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);
        } else {
            this.isFinished = true;
            drivetrain.setPoseEstimate(new Pose2d(
                    desiredTag.metadata.fieldPosition.get(0) - desiredTag.ftcPose.x - 0.0394 * 205.6,
                    desiredTag.metadata.fieldPosition.get(1) - desiredTag.ftcPose.y,
                    Math.toRadians(0) + desiredTag.ftcPose.bearing
            ));
        }
    }

    public boolean detectAprilTag() {
        boolean targetFound = false;

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                super.getTelemetry().addData("Detected ID: ", detection.id);
                if ((desiredTagId < 0) || (detection.id == desiredTagId)) {
                    targetFound = true;
                    desiredTag = detection;
                    break;
                } else {
                    super.getTelemetry().addData("Skipping: ", "Tag ID %d is not desired", detection.id);
                }
            } else {
                super.getTelemetry().addData("Unknown: ", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        super.getTelemetry().addData("Desired Tag ID: ", desiredTagId);
        return targetFound;
    }

    public void stopStreaming() {
        this.camera.stopStreaming();
        this.camera.closeCameraDevice();
    }

    public int getRegion() {
        return this.pipeline.whichRegion();
    }

    public void setDesiredTagId(int id) {
        desiredTagId = id;
    }

    public enum Color {
        RED,
        BLUE,
    }
}
