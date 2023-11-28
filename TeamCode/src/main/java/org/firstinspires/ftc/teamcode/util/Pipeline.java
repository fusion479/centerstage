package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class Pipeline extends OpenCvPipeline {
    private final Scalar red = new Scalar(255.0, 0.0, 0.0);
    Mat output = new Mat();
    Mat YCbCr = new Mat();

    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

        Rect leftRect = new Rect(1, 1, 229, 359);
        Rect midRect = new Rect(230, 1, 229, 359);
        Rect rightRect = new Rect(460, 1, 229, 359);

        input.copyTo(output);
        Imgproc.rectangle(output, leftRect, red, 2);
        Imgproc.rectangle(output, midRect, red, 2);
        Imgproc.rectangle(output, rightRect, red, 2);

        Mat leftCrop = YCbCr.submat(leftRect);
        Mat midCrop = YCbCr.submat(midRect);
        Mat rightCrop = YCbCr.submat(rightRect);

        Core.extractChannel(leftCrop, leftCrop, 2);
        Core.extractChannel(midCrop, midCrop, 2);
        Core.extractChannel(rightCrop, rightCrop, 2);

        double leftAvg = Core.mean(leftCrop).val[0];
        double midAvg = Core.mean(midCrop).val[0];
        double rightAvg = Core.mean(rightCrop).val[0];

//        if (leftAvg > midAvg && leftAvg > rightAvg) {
//            telemetry.addLine("LEFT");
//        } else if (midAvg > leftAvg && midAvg > rightAvg) {
//            telemetry.addLine("MID");
//        } else {
//            telemetry.addLine("RIGHT");
//        }

        return output;
    }
}
