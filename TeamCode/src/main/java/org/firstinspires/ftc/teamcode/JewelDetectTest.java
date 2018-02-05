package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import for_camera_opmodes.OpModeCamera;


import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Range;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import static org.opencv.imgproc.Imgproc.circle;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

//Created by Bo on 9/30/2017.

@Autonomous (name = "JewelDetect", group = "test")
public class JewelDetectTest extends OpModeCamera {

    private void startOpenCV() { //loads openCV library from phone via openCVManager
        BaseLoaderCallback openCVLoaderCallback = null;
        try {
            openCVLoaderCallback = new BaseLoaderCallback(hardwareMap.appContext) {
                @Override
                public void onManagerConnected(int status) {
                    switch (status) {
                        case LoaderCallbackInterface.SUCCESS: {
                            telemetry.addData("OpenCV", "OpenCV Manager connected!");
                        }
                        break;
                        default: {
                            super.onManagerConnected(status);
                        }
                        break;
                    }
                }
            };
        } catch (NullPointerException e) {
            telemetry.addData("Could not find OpenCV Manager!", "Please install the app from the Google Play Store.");
        }

        if (!OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_3_0, this.hardwareMap.appContext, openCVLoaderCallback)) {
            telemetry.addData("Cannot connect to OpenCV Manager", "Failure");
        }
    }

    private Mat imgMat; //holds image data
    private Bitmap img;
    private Mat CMat; //will hold circle data

    private double dp = 1; //ratio of input resolution  to output resolution

    private int loopCount = 0; //debugging purposes

    public void init() {
        startOpenCV(); //load opencvlibrary

        startCamera(); //camera init
        telemetry.addData("Camera", "Initialized");
        telemetry.update();
    }

    public void loop() {
        if (imageReady()) { //when image received from camera

            img = convertYuvImageToRgb(yuvImage, width, height, 1);
            imgMat = new Mat(new Size(img.getWidth(), img.getHeight()), CvType.CV_8UC1);

            scan4Red();
            //scan4Circles();
            //scan4CirclesB();
            telemetry.addData("Number of Circles", CMat.cols());

            if (CMat.cols() < 0) {
                if (getCircleData(CMat, 0)[0] <= img.getWidth() / 2)
                    telemetry.addData("Red Jewel on", "Right");
                else
                    telemetry.addData("Red Jewel on", "Left");
            }


            scan4Circles();
            telemetry.addData("Num of Circles", CMat.cols()); //return number of circles (# of columns = # of circles)

            if (CMat.cols() == 2) {
                if (JewelColorRED(CMat, 0)) {
                    if ((getCircleData(CMat, 0)[0]) > (getCircleData(CMat, 1)[0])) {
                        telemetry.addData("The RED jewel is to the", "RIGHT");
                    }
                    else {
                        telemetry.addData("The RED jewel is to the", "LEFT");
                    }
                }
                else {
                    if ((getCircleData(CMat, 0)[0]) > (getCircleData(CMat, 1)[0])) {
                        telemetry.addData("The BLUE jewel is to the", "RIGHT");
                    }
                    else {
                        telemetry.addData("The BLUE jewel is to the", "LEFT");
                    }
                }
            }
            else
                telemetry.addData("Circle not found", "Try Again");


            if (loopCount < 10 && CMat.cols() > 0) { //saves first 10 successful images to phone gallery
                writeToFile(imgMat, CMat);  // use this method to print circles in CMat onto the image in imgMat before saving to device
                loopCount++;
            }

        } else
            telemetry.addData("Image not loaded", "Loop count: " + loopCount);
        telemetry.update();
    }

    private void writeToFile(Mat mat, Mat circles) { //debugging only; prints images into data files on phone, access through camera roll/gallery
        // Draw the circles detected
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB, 0); //convert to rgb (so the circle that gets drawn is colored)

        int numberOfCircles = (circles.rows() == 0) ? 0 : circles.cols(); //confusing copypasta (retrieves data from mat, draw circle based on data, converts mat to bitmap, saves to system directory = Gallery)
        try {
            for (int i = 0; i < numberOfCircles; i++) {
                double[] circleCoordinates = circles.get(0, i);
                int x = (int) circleCoordinates[0];
                int y = (int) circleCoordinates[1];
                Point center = new Point(x, y);
                int r = (int) circleCoordinates[2];

                // draw the circle center
                circle(mat, center, 5, new Scalar(0, 255, 0), -1);
                // draw the circle outline
                circle(mat, center, r, new Scalar(0, 0, 255), 6);
            }
        } catch (Exception e) {
            telemetry.addData("Unable to draw circles", "Sorry");
        }
        Bitmap bmp = null;
        try {
            bmp = Bitmap.createBitmap(mat.cols(), mat.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(mat, bmp);
        } catch (CvException e) {
            telemetry.addData("Exception", "creating bitmap: " + e);
        }
        mat.release();

        FileOutputStream out = null;
        String filename = "frame" + loopCount + ".png";

        File sd = new File(Environment.getExternalStorageDirectory() + "/frames");
        boolean success = true;
        if (!sd.exists()) {
            success = sd.mkdir();
        }
        if (success) {
            File dest = new File(sd, filename);

            try {
                out = new FileOutputStream(dest);
                bmp.compress(Bitmap.CompressFormat.PNG, 100, out); // bmp is your Bitmap instance

            } catch (Exception e) {
                e.printStackTrace();
                telemetry.addData("Exception", "creating bitmap:" + e);
            } finally {
                try {
                    if (out != null) {
                        out.close();
                        telemetry.addData("File Saved", "Loop Count: " + loopCount);
                    }
                } catch (IOException e) {
                    telemetry.addData("Error", e);
                    e.printStackTrace();
                }
            }
        }
    }

    public Boolean JewelColorRED(Mat circle, int n) { //print what color each ball is (corresponds with JewelSide)
        int redValue = 0;
        int blueValue = 0;
        int pix;

        double[] list = getCircleData(circle, n);
        if (list != null) {
            for (int i = 0; i < list[2]; i++){
                pix = img.getPixel((int) list[0] - i, (int) list[1] - i);
                redValue += red(pix);
                blueValue += blue(pix);
            }
            for (int i = 0; i < list[2]; i++){
                pix = img.getPixel((int) list[0] + i, (int) list[1] + i);
                redValue += red(pix);
                blueValue += blue(pix);
            }
        }
        else {
            telemetry.addData("List is", "NULL");
            return null;
        }
        return redValue > blueValue;
    }

    private double[] getCircleData(Mat circle, int circnum) { //returns list with specific circle info; double[x, y, r]
        double[] list;
        try {
            list = circle.get(0, circnum);
            return list;
        } catch (NullPointerException e){
            telemetry.addData("No Data Found", e);
            return new double[3];
        }
    }

    private void scan4Red() { //best method that should actually work (in theory)
        Mat red1 = new Mat(new Size(img.getWidth(), img.getHeight()), CvType.CV_8UC1);
        Mat red2 = new Mat(new Size(img.getWidth(), img.getHeight()), CvType.CV_8UC1);

        Utils.bitmapToMat(img, red1);
        Utils.bitmapToMat(img, red2);

        Imgproc.cvtColor(red1, red1, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(red2, red2, Imgproc.COLOR_RGB2HSV);

        Core.inRange(red1, new Scalar(0, 100, 100), new Scalar(10, 255, 255), red1);
        Core.inRange(red2, new Scalar(160, 100, 100), new Scalar(179, 255, 255), red2);

        Mat reds = new Mat(red1.size(), CvType.CV_8UC1);

        Core.addWeighted(red1, 1, red2, 1, 0, reds);

        Imgproc.GaussianBlur(reds, reds, new Size(9, 9), 2, 2);

        CMat = new Mat(reds.size(), CvType.CV_8UC1);

        Imgproc.HoughCircles(reds, CMat, Imgproc.CV_HOUGH_GRADIENT, dp, reds.rows() / 8, 100, 35, 75, 125);

        telemetry.addData("Circle Scan", "Successful");
    }

    //alternate methods

    private void scan4Circles() { //simplified loop (probably doesn't work)
        Utils.bitmapToMat(img, imgMat);
        Imgproc.cvtColor(imgMat, imgMat, Imgproc.COLOR_RGB2GRAY, 0);
        CMat = new Mat(imgMat.size(), CvType.CV_8UC1);
        Imgproc.HoughCircles(imgMat, CMat, Imgproc.CV_HOUGH_GRADIENT, dp, imgMat.rows()/8, 100, 35, 75, 125);
        telemetry.addData("Circle Scan","Successful");
    }

    private void scan4CirclesB() { //better method that maybe works? (no)
        Mat red1 = new Mat(new Size(img.getWidth(), img.getHeight()), CvType.CV_8UC1);
        Mat red2 = new Mat(new Size(img.getWidth(), img.getHeight()), CvType.CV_8UC1);
        Mat blue1 = new Mat(new Size(img.getWidth(), img.getHeight()), CvType.CV_8UC1);
        Mat blue2 = new Mat(new Size(img.getWidth(), img.getHeight()), CvType.CV_8UC1);

        Utils.bitmapToMat(img, red1);
        Utils.bitmapToMat(img, red2);
        Utils.bitmapToMat(img, blue1);
        Utils.bitmapToMat(img, blue2);

        Imgproc.cvtColor(red1, red1, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(red2, red2, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(blue1, blue1, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(blue2, blue2, Imgproc.COLOR_RGB2HSV);

        Core.inRange(red1, new Scalar(0, 100, 100), new Scalar(10, 255, 255), red1);
        Core.inRange(red2, new Scalar(160, 100, 100), new Scalar(179, 255, 255), red2);
        Core.inRange(blue1, new Scalar(100, 150, 0), new Scalar(10, 255, 255), blue1);
        Core.inRange(blue2, new Scalar(160, 100, 100), new Scalar(179, 255, 255), blue2);

        Mat reds = new Mat(red1.size(), CvType.CV_8UC1);
        Mat blues = new Mat(blue1.size(), CvType.CV_8UC1);
        Mat total = new Mat(blue1.size(), CvType.CV_8UC1);

        Core.addWeighted(red1, 1, red2, 1, 0, reds);
        Core.addWeighted(blue1, 1, blue2, 1, 0, blues);
        Core.addWeighted(reds, 1, blues, 1, 0, total);

        Imgproc.GaussianBlur(total, total, new Size(9, 9), 2, 2);

        Imgproc.HoughCircles(total, CMat, Imgproc.CV_HOUGH_GRADIENT, dp, total.rows()/8, 100, 35, 50, 125);

        telemetry.addData("Circle Scan","Successful");
    }
} */