package org.firstinspires.ftc.teamcode;

import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.List;

import for_camera_opmodes.OpModeCamera;

/**
 * Created by Hamza Ali on 2/21/2018.
 */

@Autonomous (name = "TFTest", group = "test")
public class TFTest extends OpModeCamera {

    static {
        System.loadLibrary("tensorflow_inference");
    }

    private static final String TAG = "TFTest";

    TFClassifier network;
    AssetManager assets;

    public void init() {
        assets = hardwareMap.appContext.getAssets();
        //String path = Environment.getExternalStorageDirectory() + "/output_graph.pb";
        //String path2 = Environment.getExternalStorageDirectory() + "/labels.txt";
        //Log.i(TAG,path + " " + path);
        network = network.create(assets, "rounded_graph.pb", "retrained_labels.txt", 224, 128, 128, "input", "final_result");
        telemetry.addData("Tensorflow", "Initialized");

        startCamera(); //camera init
        telemetry.addData("Camera", "Initialized");
        telemetry.update();
    }

    public void loop() {

        if (imageReady()) { // only do this if an image has been returned from the camera

            // get image, rotated so (0,0) is in the bottom left of the preview window
            Bitmap rgbImage;
            rgbImage = convertYuvImageToRgb(yuvImage, width, height);
            List<Classifier.Recognition> results = network.recognizeImage(rgbImage);
            stopCamera();
            //network.close();
            telemetry.addData("Network Output: ", results.get(0).getTitle());
            telemetry.addData("Network Certainty: ", results.get(0).getConfidence());
            telemetry.update();
        }

        /*testing specific images: put image in pathname (must be on RC)
        Bitmap rgbImage = BitmapFactory.decodeFile(Environment.getExternalStorageDirectory() + "/TFTest.jpg");
        List<Classifier.Recognition> results = network.recognizeImage(rgbImage);
        stopCamera();
        //network.close();
        telemetry.addData("Network Output: ", results.get(0).getTitle());
        telemetry.addData("Network Certainty: ", results.get(0).getConfidence());
        telemetry.update(); */
    }
}
