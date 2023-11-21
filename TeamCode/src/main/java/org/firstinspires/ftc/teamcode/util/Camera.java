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
public class Camera extends OpenCvPipeline {
    boolean viewportPaused;

    private Mat workingMatrix = new Mat();
    private Scalar lowHSV;
    private Scalar highHSV;

    private Scalar color1[] = new Scalar[2];
    private Scalar color2[] = new Scalar[2];
    private Scalar color3[] = new Scalar[2];

    private double b1p, b2p, b3p;
    public static String c1 = "green";
    public static String c2 = "yellow";
    public static String c3 = "purple";

    private Rect ROI;

    @Override
    public final Mat processFrame(Mat input){
        //cvtColor converts a feed from a certain color format to another color format; in this case, we are converting from RGB to HSV
        Imgproc.cvtColor(input, workingMatrix, Imgproc.COLOR_RGB2HSV);
        stringToHSV(c1, color1);
        stringToHSV(c2, color2);
        stringToHSV(c3, color3);
        /*lowHSV and highHSV are our thresholds
        IMPORTANT NOTE: openCV defines HSV parameters as such (Hue, Saturation, Value) where Hue is Range 0-179,
        Saturation is in range 0-255 and Value is in range 0-255 (all INCLUSIVE).
        NORMALLY, HSV is defined like so: Hue in range 0-360, Saturation in range 0.0-1.0 and Value in range 0.0-1.0.
        All of this is also technically BGR to HSV, so take the absolute value of your Hue minus 180 to get the right number
        */


        //This creates our mask, and filters out all colors except for whats within our defined bound
            /* IGNORE ALL OF THIS FOR NOW, but essentially we'll use this to tell where our capstone is by counting pixels
            creates the submat that we want to work with
            Mat region = workingMatrix.submat(ROI);
            //this counts the number of white pixels and divides it by the area of our ROI to figure out the percentage.
            regionValue = Core.sumElems(region).val[0] / ROI.area()/255;
            //you need to release the channel that we worked with or smthn smhtn; in this case we have to release region for some reason
            region.release();
            //line color
            Scalar lines = new Scalar(25,255,255);
            //Create the rectangle so that when testing we can see the ROI that we are working with
            Imgproc.rectangle(workingMatrix,ROI,lines);*/
        ROI = new Rect(
                new Point(
                        input.cols()*(3f/8f),
                        input.rows()/4),
                new Point(
                        input.cols()*(5f/8f),
                        input.rows()*(3f/4f))
        );
        /*Imgproc.rectangle(workingMatrix, RO1, new Scalar(60, 255, 255), 10);
        Imgproc.rectangle(workingMatrix, RO2, new Scalar(60, 255, 255), 10);
        Imgproc.rectangle(workingMatrix, RO3, new Scalar(60, 255, 255), 10);*/

        //Submats for boxes, these are the regions that'll detect the color

        Mat box1 = workingMatrix.submat(ROI);
        Mat box2 = workingMatrix.submat(ROI);
        Mat box3 = workingMatrix.submat(ROI);



        Core.inRange(box1, color1[0], color1[1], box1);
        Core.inRange(box2, color2[0], color2[1], box2);
        Core.inRange(box3, color3[0], color3[1], box3);

        //How much in each region is white aka the color we filtered
        b1p = Core.sumElems(box1).val[0] / ROI.area()/255;
        b2p = Core.sumElems(box2).val[0] / ROI.area()/255;
        b3p = Core.sumElems(box3).val[0] / ROI.area()/255;

        //Compare amount of color in each region
        if(b1p > b2p && b1p > b3p) {
            Imgproc.rectangle(input, ROI, color1[1], 10);
        }else if(b2p > b1p && b2p > b3p) {
            Imgproc.rectangle(input, ROI, color2[1], 10);
        }else if(b3p > b2p & b3p > b1p) {
            Imgproc.rectangle(input, ROI, color3[1], 10);
        }
        //return the frame
        box1.release();
        box2.release();
        box3.release();
        return input;
    }

    public double color1percent() {
        return b1p * 100;
    }

    public double color2percent() {
        return b2p * 100;
    }

    public double color3percent() {
        return b3p * 100;
    }

    public int whichRegion() {
        int region = 0;
        if(b1p > b2p && b1p > b3p) {
            region = 1;
        }else if(b2p > b1p && b2p > b3p) {
            region = 2;
        }else if(b3p > b2p & b3p > b1p) {
            region = 3;
        }
        return region;
    }

    private void stringToHSV(String color, Scalar arr[]) {
        if(color.equals("green")) {
            arr[0] = new Scalar(40, 50, 50);
            arr[1] = new Scalar(75, 255, 255);
        }
        if(color.equals("red")) {
            arr[0] = new Scalar(160, 50, 50);
            arr[1] = new Scalar(180, 255, 255);
        }
        if(color.equals("blue")) {
            arr[0] = new Scalar(110, 50, 50);
            arr[1] = new Scalar(120, 255, 255);
        }
        if(color.equals("purple")) {
            arr[0] = new Scalar(135, 50, 50);
            arr[1] = new Scalar(155, 255, 255);
        }
        if(color.equals("cyan")) {
            arr[0] = new Scalar(80, 50, 50);
            arr[1] = new Scalar(95, 255, 255);
        }
        if(color.equals("yellow")) {
            arr[0] = new Scalar(20, 55, 55);
            arr[1] = new Scalar(35, 255, 255);
        }
        if(color.equals("orange")) {
            arr[0] = new Scalar(0, 50, 50);
            arr[1] = new Scalar(20, 255, 255);
        }
    }
}
