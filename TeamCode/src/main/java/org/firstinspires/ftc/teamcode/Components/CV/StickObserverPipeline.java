package org.firstinspires.ftc.teamcode.Components.CV;

import static java.lang.Math.PI;
import static java.lang.Math.sin;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class StickObserverPipeline extends OpenCvPipeline {
    double centerOfPole = 0, poleSize = 0, degPerPix = -22.5/320, widTimesDist = 16.007*58;
    ArrayList<double[]> frameList;


    public StickObserverPipeline() {
        frameList=new ArrayList<>();
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();

        //mat turns into HSV value
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        Scalar lowHSV = new Scalar(20, 70, 80); // lenient lower bound HSV for yellow
        Scalar highHSV = new Scalar(32, 255, 255); // lenient higher bound HSV for yellow

        Mat thresh = new Mat();

        // Get a black and white image of yellow objects
        Core.inRange(mat, lowHSV, highHSV, thresh);

        Mat masked = new Mat();
        //color the white portion of thresh in with HSV from mat
        //output into masked
        Core.bitwise_and(mat, mat, masked, thresh);
        //calculate average HSV values of the white thresh values
        Scalar average = Core.mean(masked,thresh);

        Mat scaledMask = new Mat();
        //scale the average saturation to 150
        masked.convertTo(scaledMask,-1,150/average.val[1],0);

        Scalar strictLowHSV = new Scalar(0, 150, 100); //strict lower bound HSV for yellow
        Scalar strictHighHSV = new Scalar(255, 255, 255); //strict higher bound HSV for yellow

        Mat scaledThresh = new Mat();

        //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
        Core.inRange(scaledMask, strictLowHSV,strictHighHSV,scaledThresh);

        Mat finalMask = new Mat();
        //color in scaledThresh with HSV(for showing result)
        Core.bitwise_and(thresh, thresh, finalMask, scaledThresh);

        Mat edges = new Mat();
        //detect edges of finalMask
        Imgproc.Canny(finalMask, edges, 100, 200);


        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        //find contours of edges
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        //rotatedRect because it allows for more accurate bounding rectangles, perfect if pole is slanted
        RotatedRect[] rectangle = new RotatedRect[contours.size()];
        //iterate through each contour
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            //convert contour to approximate polygon
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 5, true);
            //find rotatedRect for polygon
            rectangle[i] = Imgproc.minAreaRect(contoursPoly[i]);
        }

        //find index of largest rotatedRect(assumed that it is closest tile)
        int maxAreaIndex = 0;
        //iterate through each rotatedRect find largest
        for (int i = 0; i < rectangle.length; i++) {
            if (rectangle[i].size.height > rectangle[maxAreaIndex].size.height) {
                maxAreaIndex = i;
            }
        }
        //if there is a detected largest contour, record information about it
        if(rectangle.length>0) {
            centerOfPole = rectangle[maxAreaIndex].center.y + sin(rectangle[maxAreaIndex].angle) * rectangle[maxAreaIndex].size.width / 2 - 320;
            poleSize = rectangle[maxAreaIndex].size.height;
            frameList.add(new double[]{centerOfPole, poleSize});
        }
        //list of frames to reduce inconsistency, not too many so that it is still real-time
        if(frameList.size()>5) {
            frameList.remove(0);
        }
        input.release();
        edges.copyTo(input);
        scaledThresh.release();
        scaledMask.release();
        mat.release();
        masked.release();
        edges.release();
        thresh.release();
        hierarchy.release();
        finalMask.release();
        return input;
    }

    public double centerOfPole() {
        //256.227,257.307,252.9,253.414: 4,11.75|| 18.8 .073
        //2.5,12.75: 162.7,161.6, 161.7,162.5||  11.09 .068
        //2,9.5 : 187.45, 187.26|| 11.88  .0648
         //1,8,7.8 : 273 || 12.99 .047
        //10.6,22.2 :
        //4.1,20.6 :
        double average=0;
        for(int i=0;i<frameList.size();i++){
            average+=frameList.get(i)[0];
        }
        return average/frameList.size();
    }

    public double poleSize() {
        double average=0;
        for(int i=0;i<frameList.size();i++){
            average+=frameList.get(i)[1];
        }
        return average/frameList.size();
    }

    public double[] poleRotatedPolarCoordDelta() {
        return new double[]{degPerPix * centerOfPole() * PI / 180, widTimesDist / poleSize()};
    }
}