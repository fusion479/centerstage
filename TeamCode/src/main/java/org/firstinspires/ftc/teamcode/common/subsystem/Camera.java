package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.common.util.Pipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class Camera extends Mechanism {
    public static int DESIRED_TAG_ID = 2;
    final double DESIRED_DISTANCE = 5;
    final double SPEED_GAIN = 0.03;
    final double STRAFE_GAIN = 0.025;
    final double TURN_GAIN = 0.03;
    final double MAX_AUTO_SPEED = 0.3;
    final double MAX_AUTO_STRAFE = 0.3;
    private final String color;
    double MAX_AUTO_TURN = 0.15;
    OpenCvCamera openCvCamera;
    Pipeline pipeline;
    private boolean isFinished = false;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    public Camera(String color) {
        this.color = color;
        pipeline = new Pipeline(color);
    }

    public boolean getFinished() {
        return isFinished;
    }

    @Override
    public void init(HardwareMap hwMap) {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        openCvCamera = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        openCvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                openCvCamera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(openCvCamera, 0);
                openCvCamera.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {
                // when the camera cannot be opened
            }
        });
    }

    public void aprilTagInit(HardwareMap hwMap, int region) {
        if (color.equals("blue")) {
            DESIRED_TAG_ID = region;
        } else if (color.equals("red")) {
            DESIRED_TAG_ID = region + 3;
        }

        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(3);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "camera"))
                .addProcessor(aprilTag)
                .build();
    }

    public void stopStreaming() {
        openCvCamera.stopStreaming();
        openCvCamera.closeCameraDevice();
    }

    public int whichRegion() {
        return pipeline.whichRegion();
    }

    public void moveRobot(SampleMecanumDrive drivetrain, Telemetry telemetry) {
        double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
        double headingError = desiredTag.ftcPose.bearing;
        double yawError = desiredTag.ftcPose.yaw;

        double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
        double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        telemetry.addData("Errors", "Range %5.2f, YAW %5.2f, Heading %5.2f", rangeError, headingError, yawError);

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


        if (((Math.abs(rangeError) + Math.abs(yawError)) / 2) > 0.15 && Math.abs(headingError) > 0.05) {
            drivetrain.setMotorPowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);
            isFinished = false;
        } else {
            // EXPERIMENTAL, REMOVE IF EVERYTHING BREAKS
            // EXPERIMENTAL, REMOVE IF EVERYTHING BREAKS
            // EXPERIMENTAL, REMOVE IF EVERYTHING BREAKS
            drivetrain.setPoseEstimate(new Pose2d(
                    desiredTag.metadata.fieldPosition.get(0) - desiredTag.ftcPose.range - 6,
                    desiredTag.metadata.fieldPosition.get(1),
                    Math.toRadians(desiredTag.ftcPose.yaw)));
            
            isFinished = true;
        }
    }

    public boolean detectAprilTag(Telemetry telemetry) {
        boolean targetFound = false;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addData("detected ID", detection.id);
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    targetFound = true;
                    desiredTag = detection;
                    break;
                } else {
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        telemetry.addData("DESIRED TAG", DESIRED_TAG_ID);
        return targetFound;
    }

    public void setManualExposure(int exposureMS, int gain, boolean isStopRequested, Telemetry telemetry, LinearOpMode opMode) {
        if (visionPortal == null) {
            return;
        }
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                opMode.sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
        if (!isStopRequested) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                opMode.sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            opMode.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            opMode.sleep(20);
        }
    }
}
