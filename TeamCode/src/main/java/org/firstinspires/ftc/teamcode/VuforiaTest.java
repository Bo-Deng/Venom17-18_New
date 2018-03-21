package org.firstinspires.ftc.teamcode;

/**
 * Created by Bo on 2/28/2017.
 */

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.*;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Arrays;

import for_camera_opmodes.LinearOpModeCamera;
import for_camera_opmodes.OpModeCamera;

@Autonomous(name = "VuforiaTest", group = "Testa")
public class VuforiaTest extends LinearOpMode {

    ClosableVuforiaLocalizer vuforia;
    ClosableVuforiaLocalizer.Parameters parameters;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    RelicRecoveryVuMark vuMark;
    char template;

    public void runOpMode() {
        //vuforia init
        telemetry.addLine("Vuforia initializing!");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AXb/g5n/////AAAAGSUed2rh5Us1jESA1cUn5r5KDUqTfwO2woh7MxjiLKSUyDslqBAgwCi0Qmc6lVczErnF5TIw7vG5R4TJ2igvrDVp+dP+3i2o7UUCRRj/PtyVgb4ZfNrDzHE80/6TUHifpKu4QCM04eRWYZocWNWhuRfytVeWy6NSTWefM9xadqG8FFrFk3XnvqDvk/6ZAgerNBdq5SsJ90eDdoAhgYEee40WxasoUUM9YVMvkWOqZgHSuraV2IyIUjkW/u0O+EkFtTNRUWP+aZwn1qO1H4Lk07AJYe21eqioBLMdzY7A8YqR1TeQ//0WJg8SFdXjuGbF6uHykBe2FF5UeyaehA0iTqfPS+59FLm8y1TuUt57eImq";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforia = new ClosableVuforiaLocalizer(parameters);

        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addLine("Vuforia init complete");
        telemetry.update();

        //loop vuforia until end of init
        relicTrackables.activate();
        while (!opModeIsActive()) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("VuMark ", vuMark);
            telemetry.update();
            sleep(100);
        }

        waitForStart();

        //vumark should be determined during loop
        telemetry.addData("VuMark ", vuMark);
        if (vuMark == RelicRecoveryVuMark.CENTER)
            template = 'C';
        else if (vuMark == RelicRecoveryVuMark.LEFT)
            template = 'L';
        else if (vuMark == RelicRecoveryVuMark.RIGHT)
            template = 'R';
        vuforia.close(); //hopefully close vuforia

        telemetry.addData("Vumark Detected", template);
        telemetry.update();
    }
/*
    public Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int pixelFormat) {
        long numImgs = frame.getNumImages();

        for (int i = 0; i < numImgs; i++) {
            if (frame.getImage(i).getFormat() == pixelFormat)
                return frame.getImage(i);
        }
        return null;
    }
    public boolean compareToVumark(Image img, org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark Vumark) {
        if (img.equals(Vumark)) {
            return true;
        }
        return false;
    } */
}

