package org.firstinspires.ftc.teamcode.util;

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
    String color;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();

    Mat mat = new Mat();
    Scalar lowHSV;
    Scalar highHSV;
    Rect ROI1, ROI2, ROI3;
    double region1Percent, region2Percent, region3Percent;
    int region;

    Rect leftRect = new Rect(1, 1, 213, 479);
    Rect midRect = new Rect(214, 1, 213, 479);
    Rect rightRect = new Rect(427, 1, 213, 479);

    public Pipeline(String colorChoice) {
        color = colorChoice;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        if (color == "red") {
            lowHSV = new Scalar(160, 50, 50);
            highHSV = new Scalar(180, 255, 255);
        } if (color == "blue") {
            lowHSV = new Scalar(110, 50, 50);
            highHSV = new Scalar(120, 255, 255);
        }

        ROI1 = new Rect(1, 1, 213, 479);
        ROI2 = new Rect(214, 1, 213, 479);
        ROI3 = new Rect(427, 1, 213, 479);

        Core.inRange(mat, lowHSV, highHSV, mat);

        // submats for the boxes, these are the regions that'll detect the color
        Mat box1 = mat.submat(ROI1);
        Mat box2 = mat.submat(ROI2);
        Mat box3 = mat.submat(ROI3);

        // how much in each region is white aka the color we filtered through the mask
        region1Percent = Core.sumElems(box1).val[0] / ROI1.area() / 255;
        region2Percent = Core.sumElems(box2).val[0] / ROI2.area() / 255;
        region3Percent = Core.sumElems(box3).val[0] / ROI3.area() / 255;

        telemetry.addData("region1", region1Percent);
        telemetry.addData("region2", region2Percent);
        telemetry.addData("region3", region3Percent);
        telemetry.update();

        if (region1Percent > region2Percent && region1Percent > region3Percent) {
            Imgproc.rectangle(mat, ROI1, new Scalar(60, 255, 255), 10);
            region = 1;
        } else if (region2Percent > region1Percent && region2Percent > region3Percent) {
            Imgproc.rectangle(mat, ROI2, new Scalar(60, 255, 255), 10);
            region = 2;
        } else if (region3Percent > region1Percent && region3Percent > region2Percent) {
            Imgproc.rectangle(mat, ROI3, new Scalar(60, 255, 255), 10);
            region = 3;
        }

        box1.release();
        box2.release();
        box3.release();

        return mat;
    }

    int whichRegion() {
        return region;
    }
}
