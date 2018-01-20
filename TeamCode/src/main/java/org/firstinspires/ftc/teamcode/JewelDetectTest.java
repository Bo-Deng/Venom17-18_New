package org.firstinspires.ftc.teamcode;

/*
import android.graphics.Bitmap;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import for_camera_opmodes.OpModeCamera;


import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
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

    public void startOpenCV() { //loads openCV library from phone via openCVManager
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

    public Mat imgMat; //Mat = matrix
    public Mat CMat; //holds image data

    private double dp = 1; //ratio of input resolution  to output resolution
    private double minDst = 100; //min distance between centers of detected circles

    int loopCount = 0; //debugging purposes

    public void init() {
        startOpenCV(); //load opencvlibrary

        startCamera(); //camera init
        telemetry.addData("Camera", "Initialized");
        telemetry.update();
    }

    public void loop() {
        if (imageReady()) { //when image received from camera

            scanCircles();

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
            telemetry.update();

            if (loopCount < 10 && CMat.cols() == 2) { //saves first 10 successful images to phone gallery
                writeToFile(imgMat, CMat);  // use this method to print circles in CMat onto the image in imgMat before saving to device
                loopCount++;
            }

        } else {
            telemetry.addData("Image not loaded", "Loop count: " + loopCount);
            telemetry.update();
        }
    }

    public void writeToFile(Mat mat, Mat circles) { //debugging only; prints images into data files on phone, access through camera roll/gallery

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
        Bitmap img = convertYuvImageToRgb(yuvImage, width, height, 1);
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
        if (redValue > blueValue)
            return true;
        return false;
    }

    public double[] getCircleData(Mat circle, int circnum) { //returns list with specific circle info
        double[] list;
        try {
            list = circle.get(0, circnum);
            return list;
        } catch (NullPointerException e){
            telemetry.addData("No Data Found", e);
            return null;
        }
    }

    public void scanCircles() { //simplified loop
        if (imageReady()) {
            Bitmap img = convertYuvImageToRgb(yuvImage, width, height, 1);
            imgMat = new Mat(new Size(img.getWidth(), img.getHeight()), CvType.CV_8UC1);
            Utils.bitmapToMat(img, imgMat);
            Imgproc.cvtColor(imgMat, imgMat, Imgproc.COLOR_RGB2GRAY, 0);
            CMat = new Mat(imgMat.size(), CvType.CV_8UC1);
            Imgproc.HoughCircles(imgMat, CMat, Imgproc.CV_HOUGH_GRADIENT, dp, minDst, 70, 35, 75, 125);
            telemetry.addData("Image Scan","Successful");
            telemetry.update();
        } else {
            telemetry.addData("Image Scan","Failure");
            telemetry.update();
        }
    }
} */