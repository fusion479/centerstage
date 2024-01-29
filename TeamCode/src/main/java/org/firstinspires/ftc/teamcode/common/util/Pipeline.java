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
    Rect RIGHT_RECT, CENTER_RECT;
    double rightRegionPercent, centerRegionPercent;
    int region;

    public Pipeline(String colorChoice) {
        color = colorChoice;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        if (color == "red") {
            lowHSV = new Scalar(140, 50, 50);
            highHSV = new Scalar(180, 255, 255);
        }
        if (color == "blue") {
            lowHSV = new Scalar(110, 50, 50);
            highHSV = new Scalar(120, 255, 255);
        }

        RIGHT_RECT = new Rect(213, 1, 213, 239);
        CENTER_RECT = new Rect(213, 240, 213, 239);

        Core.inRange(mat, lowHSV, highHSV, mat);

        // submats for the boxes, these are the regions that'll detect the color
        Mat box1 = mat.submat(RIGHT_RECT);
        Mat box2 = mat.submat(CENTER_RECT);

        // how much in each region is white aka the color we filtered through the mask
        rightRegionPercent = Core.sumElems(box1).val[0] / RIGHT_RECT.area() / 255;
        centerRegionPercent = Core.sumElems(box2).val[0] / CENTER_RECT.area() / 255;

        telemetry.addData("region1", rightRegionPercent);
        telemetry.addData("region2", centerRegionPercent);
        telemetry.update();

        if (rightRegionPercent < r3threshold && centerRegionPercent < r3threshold) {
            region = 1;
        } else if (rightRegionPercent > centerRegionPercent) {
            Imgproc.rectangle(mat, RIGHT_RECT, new Scalar(60, 255, 255), 10);
            region = 3;
        } else if (centerRegionPercent > rightRegionPercent) {
            Imgproc.rectangle(mat, CENTER_RECT, new Scalar(60, 255, 255), 10);
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
