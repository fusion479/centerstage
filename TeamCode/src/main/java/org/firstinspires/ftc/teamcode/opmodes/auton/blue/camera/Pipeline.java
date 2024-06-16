package org.firstinspires.ftc.teamcode.opmodes.auton.blue.camera;

import com.acmerobotics.dashboard.config.Config;

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

    public static int rightRectX = 565;
    public static int rightRectY = 125;

    public static int leftRectX = 0;
    public static int leftRectY = 125;
    public static double tolerance = 0.1;
    private final Camera.Color color;
    int region;
    private final Mat mat = new Mat();
    private Scalar lowFilter, highFilter;
    private Rect RIGHT_RECT, LEFT_RECT;

    public Pipeline(Camera.Color color) {
        this.color = color;
    }

    @Override
    public Mat processFrame(Mat input) {
        // turn image to HSV to reduce illumination noise
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        if (color == Camera.Color.RED) {
            lowFilter = new Scalar(redLowHSVR, redLowHSVG, redLowHSVB);
            highFilter = new Scalar(redHighHSVR, redHighHSVG, redHighHSVB);
        } else {
            lowFilter = new Scalar(blueLowHSVR, blueLowHSVG, blueLowHSVB);
            highFilter = new Scalar(blueHighHSVR, blueHighHSVG, blueHighHSVB);
        }

        RIGHT_RECT = new Rect(rightRectX, rightRectY, 75, 125);
        LEFT_RECT = new Rect(leftRectX, leftRectY, 75, 125);

        Core.inRange(mat, lowFilter, highFilter, mat);

        Mat rightBox = mat.submat(RIGHT_RECT);
        Mat leftBox = mat.submat(LEFT_RECT);

        Imgproc.rectangle(mat, LEFT_RECT, new Scalar(60, 255, 255), 5);
        Imgproc.rectangle(mat, RIGHT_RECT, new Scalar(60, 255, 255), 5);

        /*
            REGION 1: LEFT
            REGION 2: CENTER
            REGION 3: RIGHT
        */

        if (Core.sumElems(leftBox).val[0] / LEFT_RECT.area() / 255 > tolerance) {
            Imgproc.rectangle(mat, LEFT_RECT, new Scalar(60, 255, 255), 10);
            region = 1;
        } else if (Core.sumElems(rightBox).val[0] / RIGHT_RECT.area() / 255 > tolerance) {
            Imgproc.rectangle(mat, RIGHT_RECT, new Scalar(60, 255, 255), 10);
            region = 3;
        } else {
            region = 2;
        }

        rightBox.release();
        leftBox.release();

        return mat;
    }

    public int whichRegion() {
        return region;
    }
}