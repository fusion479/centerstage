package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class TestCamera extends OpMode {
    OpenCvWebcam webcam;

    @Override
    public void init() {
        WebcamName name = hardwareMap.get(WebcamName.class, "webcam1");
        int id = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.webcam = OpenCvCameraFactory.getInstance().createWebcam(name, id);

        this.webcam.setPipeline(new MainPipeline());

        this.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(690, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Error occured while initializing camera.");
            }
        });
    }

    @Override
    public void loop() {}


    private class MainPipeline extends OpenCvPipeline {
        private final Scalar red = new Scalar(255.0, 0.0, 0.0);

        public Mat processFrame(Mat input) {
            Mat output = new Mat();
            Mat YCbCr = new Mat();

            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

            Rect leftRect = new Rect(1, 1, 229, 359);
            Rect midRect = new Rect(230, 1, 229, 359);
            Rect rightRect = new Rect(460, 1, 229, 359);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, red, 2);
            Imgproc.rectangle(output, midRect, red, 2);
            Imgproc.rectangle(output, rightRect, red, 2);

            Mat leftCrop =  YCbCr.submat(leftRect);
            Mat midCrop = YCbCr.submat(midRect);
            Mat rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(midCrop, midCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);

            double leftAvg = Core.mean(leftCrop).val[0];
            double midAvg = Core.mean(midCrop).val[0];
            double rightAvg = Core.mean(rightCrop).val[0];

            if (leftAvg > midAvg && leftAvg > rightAvg) {
                telemetry.addLine("LEFT");
            } else if (midAvg > leftAvg && midAvg > rightAvg) {
                telemetry.addLine("MID");
            } else {
                telemetry.addLine("RIGHT");
            }

            return output;
        }
    }
}
