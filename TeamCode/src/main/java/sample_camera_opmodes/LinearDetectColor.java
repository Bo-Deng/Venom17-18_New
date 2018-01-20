package sample_camera_opmodes;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import for_camera_opmodes.LinearOpModeCamera;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */

@TeleOp(name = "LinearDetectColor", group = "ZZOpModeCameraPackage")
//@Disabled
public class LinearDetectColor extends LinearOpModeCamera {


    int ds2 = 1;  // additional downsampling of the image
    // set to 1 to disable further downsampling

    @Override
    public void runOpMode() {

        telemetry.addLine("startJewelCamera initialization started");
        telemetry.update();

        setCameraDownsampling(2);

        telemetry.addLine("Wait for camera to finish initializing!");

        startCamera();  // can take a while.
        // best started before waitForStart
        sleep(2000);
        telemetry.addLine("Camera ready!");

        ElapsedTime times = new ElapsedTime();

        times.reset();
        int numPics = 0;
        int redValue = 0;
        int blueValue = 0;
        int numFailLoops = 0;

        while (times.seconds() < 5) {
            if (imageReady()) { // only do this if an image has been returned from the camera

                numPics++;

                // get image, rotated so (0,0) is in the bottom left of the preview window
                Bitmap rgbImage;
                rgbImage = convertYuvImageToRgb(yuvImage, width, height, 1);

                for (int x = (int) (.8 * rgbImage.getWidth()); x < rgbImage.getWidth(); x++) {
                    for (int y = 0; y < (int) (.25 * rgbImage.getHeight()); y++) {
                        int pixel = rgbImage.getPixel(x, y);
                        redValue += red(pixel);
                        blueValue += blue(pixel);
                    }
                }
            }
            else {
                numFailLoops++;
            }

            sleep(10);
        }

        boolean jewelIsRed = redValue > blueValue;
        telemetry.addData("Is Jewel Red?", jewelIsRed);
        telemetry.addData("numPics: ", numPics);
        telemetry.addData("numFailLoops: ", numFailLoops);
        telemetry.addData("red blue: ", redValue + "    " + blueValue);
        telemetry.update();

        stopCamera();
            /*
            setCameraDownsampling(8);
            // parameter determines how downsampled you want your images
            // 8, 4, 2, or 1.
            // higher number is more downsampled, so less resolution but faster
            // 1 is original resolution, which is detailed but slow
            // must be called before super.init sets up the camera

            telemetry.addLine("Wait for camera to finish initializing!");
            telemetry.update();
            startCamera();  // can take a while.
            // best started before waitForStart
            telemetry.addLine("Camera ready!");
            telemetry.update();*/

            waitForStart();

            // LinearOpMode, so could do stuff like this too.
            /*
            motorLeft.setPower(1);  // drive forward
            motorRight.setPower(1);
            sleep(1000);            // for a second.
            motorLeft.setPower(0);  // stop drive motors.
            motorRight.setPower(0);
            sleep(1000);            // wait a second.
            */

            while (opModeIsActive()) {

            }
            stopCamera();
    }
}

