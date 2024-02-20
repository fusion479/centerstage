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
    public static int blueLowHSVR = 40;
    public static int blueLowHSVG = 40;
    public static int blueLowHSVB = 0;
    public static int blueHighHSVR = 180;
    public static int blueHighHSVG = 255;
    public static int blueHighHSVB = 180;
    public static int redLowHSVR = 0;
    public static int redLowHSVG = 120;
    public static int redLowHSVB = 0;
    public static int redHighHSVR = 255;
    public static int redHighHSVG = 255;
    public static int redHighHSVB = 255;
    public double tolerance = 0.3;

    String color;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();
    Mat mat = new Mat();
    Scalar lowHSV;
    Scalar highHSV;
    Rect RIGHT_RECT, CENTER_RECT, LEFT_RECT;
    double rightRegionPercent, centerRegionPercent, leftRegionPercent;
    int region;


    public Pipeline(String colorChoice) {
        color = colorChoice;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        if (color == "red") {
            lowHSV = new Scalar(redLowHSVR, redLowHSVG, redLowHSVB);
            highHSV = new Scalar(redHighHSVR, redHighHSVG, redHighHSVB);
        }
        if (color == "blue") {
            lowHSV = new Scalar(blueLowHSVR, blueLowHSVG, blueLowHSVB);
            highHSV = new Scalar(blueHighHSVR, blueHighHSVG, blueHighHSVB);
        }

        RIGHT_RECT = new Rect(500, 200, 125, 125);
        CENTER_RECT = new Rect(250, 200, 125, 125);
        LEFT_RECT = new Rect(0, 200, 125, 125);

        Core.inRange(mat, lowHSV, highHSV, mat);

        // submats for the boxes, these are the regions that'll detect the color
        Mat rightBox = mat.submat(RIGHT_RECT);
        Mat centerBox = mat.submat(CENTER_RECT);
        Mat leftBox = mat.submat(LEFT_RECT);

        // how much in each region is white aka the color we filtered through the mask
        rightRegionPercent = Core.sumElems(rightBox).val[0] / RIGHT_RECT.area() / 255;
        centerRegionPercent = Core.sumElems(centerBox).val[0] / CENTER_RECT.area() / 255;
        leftRegionPercent = Core.sumElems(leftBox).val[0] / LEFT_RECT.area() / 255;

        telemetry.addData("right region", rightRegionPercent);
        telemetry.addData("center region", centerRegionPercent);
        telemetry.addData("left region", leftRegionPercent);
        telemetry.update();

        Imgproc.rectangle(mat, LEFT_RECT, new Scalar(60, 255, 255), 5);
        Imgproc.rectangle(mat, RIGHT_RECT, new Scalar(60, 255, 255), 5);
        Imgproc.rectangle(mat, CENTER_RECT, new Scalar(60, 255, 255), 5);

        if (leftRegionPercent > tolerance && leftRegionPercent > centerRegionPercent && leftRegionPercent > rightRegionPercent) {
            Imgproc.rectangle(mat, LEFT_RECT, new Scalar(60, 255, 255), 10);
            region = 1;
        } else if (rightRegionPercent > tolerance && rightRegionPercent > centerRegionPercent && rightRegionPercent > leftRegionPercent) {
            Imgproc.rectangle(mat, RIGHT_RECT, new Scalar(60, 255, 255), 10);
            region = 3;
        } else { // (centerRegionPercent > tolerance && centerRegionPercent > rightRegionPercent && centerRegionPercent > leftRegionPercent) {
            Imgproc.rectangle(mat, CENTER_RECT, new Scalar(60, 255, 255), 10);
            region = 2;
        }

        rightBox.release();
        leftBox.release();
        centerBox.release();

        return mat;
    }

    public int whichRegion() {
        return region;
    }
}
