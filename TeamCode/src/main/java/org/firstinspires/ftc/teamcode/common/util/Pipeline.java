package org.firstinspires.ftc.teamcode.common.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class Pipeline extends OpenCvPipeline {
    public double r3threshold = .05;
    String color;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();
    Mat mat = new Mat();
    Scalar lowHSV;
    Scalar highHSV;
    Rect ROI1, ROI2;
    double region1Percent, region2Percent;
    int region;

    public Pipeline(String colorChoice) {
        color = colorChoice;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        if (color == "red") {
            lowHSV = new Scalar(160, 50, 50);
            highHSV = new Scalar(180, 255, 255);
        }
        if (color == "blue") {
            lowHSV = new Scalar(110, 50, 50);
            highHSV = new Scalar(120, 255, 255);
        }

        ROI1 = new Rect(213, 1, 213, 239);
        ROI2 = new Rect(213, 240, 213, 239);

        Core.inRange(mat, lowHSV, highHSV, mat);

        // submats for the boxes, these are the regions that'll detect the color
        Mat box1 = mat.submat(ROI1);
        Mat box2 = mat.submat(ROI2);

        // how much in each region is white aka the color we filtered through the mask
        region1Percent = Core.sumElems(box1).val[0] / ROI1.area() / 255;
        region2Percent = Core.sumElems(box2).val[0] / ROI2.area() / 255;

        telemetry.addData("region1", region1Percent);
        telemetry.addData("region2", region2Percent);
        telemetry.update();

        if (region1Percent < r3threshold && region2Percent < r3threshold) {
            region = 3;
        } else if (region1Percent > region2Percent) {
            Imgproc.rectangle(mat, ROI1, new Scalar(60, 255, 255), 10);
            region = 1;
        } else if (region2Percent > region1Percent) {
            Imgproc.rectangle(mat, ROI2, new Scalar(60, 255, 255), 10);
            region = 2;
        }

        box1.release();
        box2.release();

        return mat;
    }

    public int whichRegion() {
        return region;
    }
}
