package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbServoController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.IOException;

import for_camera_opmodes.LinearOpModeCamera;

/**
 * Created by Bo on 9/13/2017.
 */
public class CustomLinearOpMode extends LinearOpModeCamera {
    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;

    IMU imu;
    ModernRoboticsI2cRangeSensor rangeSensorL;
    ModernRoboticsI2cRangeSensor rangeSensorR;
    //AnalogInput button;

    Servo servoLLHug;
    Servo servoLRHug;
    Servo servoULHug;
    Servo servoURHug;
    Servo servoFlip;

    Servo servoUpDownArm;
    Servo servoLeftRightArm;

    DcMotor motorLiftL;
    DcMotor motorLiftR;

    String AutoColor;
    char template;
    boolean jewelIsRed;

    double sf = 1.3;

    int squaresToEncoder = (int) (1120 * sf); //use motorBL

    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    ClosableVuforiaLocalizer vuforia;

    ElapsedTime times;

    //left hug variables
    /*double leftOpenPos = .649;
    double leftThreadPos = .32;
    double leftClampPos = .18;
    //right hug variables
    double rightOpenPos = .43;
    double rightThreadPos = .8;
    double rightClampPos = 1.0;*/

    double LLOpen = .54;
    double LLClose = .95;
    double LLThread = .85;

    double ULOpen = 1.0;
    double ULClose = .5;
    double ULThread = .6;

    double LROpen = .595;
    double LRClose = .15;
    double LRThread = .25;

    double UROpen = 0.005;
    double URClose = .4;
    double URThread = .3;


    //vuforia
    ClosableVuforiaLocalizer.Parameters parameters;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    RelicRecoveryVuMark vuMark;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void initStuff(HardwareMap map) throws InterruptedException {

        times = new ElapsedTime();

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


        motorFR = map.dcMotor.get("motorFR");
        motorFL = map.dcMotor.get("motorFL");
        motorBR = map.dcMotor.get("motorBR");
        motorBL = map.dcMotor.get("motorBL");

        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLiftL = map.dcMotor.get("motorLiftL");
        motorLiftR = map.dcMotor.get("motorLiftR");

        rangeSensorL = map.get(ModernRoboticsI2cRangeSensor.class, "rangeL");
        rangeSensorR = map.get(ModernRoboticsI2cRangeSensor.class, "rangeR");
        //button = hardwareMap.get(AnalogInput.class, "button");

        //This is right i promise don't worry about it
        servoLLHug = map.servo.get("servoLRHug");
        servoLRHug = map.servo.get("servoLLHug");
        servoULHug = map.servo.get("servoURHug");
        servoURHug = map.servo.get("servoULHug");
        servoFlip = map.servo.get("servoFlip");

        servoLeftRightArm = map.servo.get("servoLeftRightArm");
        servoUpDownArm = map.servo.get("servoUpDownArm");

        imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
        imu.IMUinit(hardwareMap);

        servoLLHug.setPosition(LLOpen);
        servoLRHug.setPosition(LROpen);
        servoULHug.setPosition(ULOpen);
        servoURHug.setPosition(UROpen);

        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
        }

        servoFlip.setPosition(.025);
        servoUpDownArm.setPosition(1);
        servoLeftRightArm.setPosition(.40);

        telemetry.addData("PID value = ", ".0275");
        telemetry.addData("init = ", "completed");
        telemetry.update();

        template = 'R';

        //loop until play button pressed
        relicTrackables.activate();
        while (!opModeIsActive()) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark == RelicRecoveryVuMark.CENTER) {
                template = 'C';
            }
            if (vuMark == RelicRecoveryVuMark.LEFT) {
                template = 'L';
            }
            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                template = 'R';
            }
        }
    }

    public void getJewelColor() {
        //jewel camera init
        telemetry.addLine("JewelCamera initialization started");
        telemetry.update();

        setCameraDownsampling(2);

        telemetry.addLine("Wait for camera to finish initializing!");
        telemetry.update();

        startCamera();  // can take a while.

        sleep(50);

        telemetry.addLine("Camera ready!");
        telemetry.update();

        times.reset();
        int numPics = 0;
        int redValue = 0;
        int blueValue = 0;
        int numFailLoops = 0;

        while (times.seconds() < 2 && opModeIsActive()) {
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
            } else {
                numFailLoops++;
            }

            sleep(10);
        }

        jewelIsRed = redValue > blueValue;

        stopCamera();

        telemetry.addData("Is Jewel Red?", jewelIsRed);

        telemetry.addData("numPics: ", numPics);
        telemetry.addData("numFailLoops: ", numFailLoops);
        telemetry.addData("red blue: ", redValue + "    " + blueValue);
    }

    public void getVuMark() {
        //vumark should be determined during loop
        telemetry.addData("VuMark ", vuMark);
        if (vuMark == RelicRecoveryVuMark.CENTER)
            template = 'C';
        else if (vuMark == RelicRecoveryVuMark.LEFT)
            template = 'L';
        else if (vuMark == RelicRecoveryVuMark.RIGHT)
            template = 'R';
        vuforia.close(); //hopefully close vuforia
    }

    public void stopMotors() {
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    public void startMotors(double speed) {
        motorFL.setPower(speed);
        motorFR.setPower(speed);
        motorBL.setPower(speed);
        motorBR.setPower(speed);
    }

    public void setMode(DcMotor.RunMode runMode) throws InterruptedException {
        motorFR.setMode(runMode);
        idle();
        motorFL.setMode(runMode);
        idle();
        motorBR.setMode(runMode);
        idle();
        motorBL.setMode(runMode);
        idle();
    }

    public void strafe(int d, double p) throws InterruptedException { // d = direction, p = power

        if (d == 0) { //left
            motorFL.setPower(-p);
            motorFR.setPower(p);
            motorBL.setPower(p);
            motorBR.setPower(-p);
        } else if (d == 1) { //right
            motorFL.setPower(p);
            motorFR.setPower(-p);
            motorBL.setPower(-p);
            motorBR.setPower(p);
        }
    }

    public double getRightDistance() {
        double dist = rangeSensorR.getDistance(DistanceUnit.CM);
        while (dist > 1000 || Double.isNaN(dist) && opModeIsActive()) {
            dist = rangeSensorR.getDistance(DistanceUnit.CM);
        }
        return dist;
    }

    public double getLeftDistance() {
        double dist = rangeSensorL.getDistance(DistanceUnit.CM);
        while (dist > 1000 || Double.isNaN(dist) && opModeIsActive()) {
            dist = rangeSensorL.getDistance(DistanceUnit.CM);
        }
        return dist;
    }

    public String getColor() {
        String colorString = "NONE";

        // linear OpMode, so could do stuff like this too.
        /*
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        */

        if (isCameraAvailable()) {

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
            telemetry.update();


            // LinearOpMode, so could do stuff like this too.
            /*
            motorLeft.setPower(1);  // drive forward
            motorRight.setPower(1);
            sleep(1000);            // for a second.
            motorLeft.setPower(0);  // stop drive motors.
            motorRight.setPower(0);
            sleep(1000);            // wait a second.
            */

            int ds2 = 2;

            while (opModeIsActive()) {
                if (imageReady()) { // only do this if an image has been returned from the camera
                    int redValue = 0;
                    int blueValue = 0;
                    int greenValue = 0;

                    // get image, rotated so (0,0) is in the bottom left of the preview window
                    Bitmap rgbImage;
                    rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);

                    for (int x = 0; x < rgbImage.getWidth(); x++) {
                        for (int y = 0; y < (int) (.25 * rgbImage.getHeight()); y++) {
                            int pixel = rgbImage.getPixel(x, y);
                            redValue += red(pixel);
                            blueValue += blue(pixel);
                            greenValue += green(pixel);
                        }
                    }
                    int color = highestColor(redValue, greenValue, blueValue);

                    switch (color) {
                        case 0:
                            colorString = "RED";
                            break;
                        case 1:
                            colorString = "GREEN";
                            break;
                        case 2:
                            colorString = "BLUE";
                    }

                } else {
                    colorString = "NONE";
                }

                telemetry.addData("Color:", "Color detected is: " + colorString);
                telemetry.update();
                sleep(10);
            }
            stopCamera();


        }
        return colorString;
    }

    public void moveSquares(double squares, double power) throws InterruptedException {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (squares > 0) {
            while (Math.abs(motorBL.getCurrentPosition()) < (squares * squaresToEncoder) && opModeIsActive()) {
                startMotors(power);
            }
        } else {
            while (-Math.abs(motorBL.getCurrentPosition()) > (squares * squaresToEncoder) && opModeIsActive()) {
                startMotors(-power);
                //telemetry.addData("Encoder: " + -Math.abs(motorBL.getCurrentPosition()), "out of : " + squares * squaresToEncoder);
                //telemetry.update();
            }
        }
        stopMotors();
    }

    public void moveTime(int msTime, double power) throws InterruptedException {
        setMotors(power, power, power, power);
        sleep(msTime);
        stopMotors();
    }

    public void wiggle(double power, double angle) throws InterruptedException {
        setMotors(power, power, power, power);
        sleep(500);
        stopMotors();


        strafeLeft(.4, angle);
        //setMotors(motorFL.getPower() + .3, motorBL.getPower() + .3, motorFR.getPower() + .3, motorBR.getPower() + .3);
        sleep(500);
        stopMotors();

        setMotors(power, power, power, power);
        sleep(400);
        stopMotors();


        strafeRight(.4, angle);
        //setMotors(motorFL.getPower() + .3, motorBL.getPower() + .3, motorFR.getPower() + .3, motorBR.getPower() + .3);
        sleep(1000);
        stopMotors();

        setMotors(power, power, power, power);
        sleep(400);
        stopMotors();

        strafeLeft(.4, angle);
        sleep(500);
        stopMotors();
    }

    public void wiggleNoRight(double power, double angle) throws InterruptedException {
        setMotors(power, power, power, power);
        sleep(400);
        stopMotors();


        strafeLeft(.4, angle);
        //setMotors(motorFL.getPower() + .3, motorBL.getPower() + .3, motorFR.getPower() + .3, motorBR.getPower() + .3);
        sleep(500);
        stopMotors();

        setMotors(power, power, power, power);
        sleep(400);
        stopMotors();

    }

    public void setMotors(double FLpow, double BLpow, double FRpow, double BRpow) {
        motorFL.setPower(Range.clip(FLpow, -1, 1));
        motorBL.setPower(Range.clip(BLpow, -1, 1));
        motorFR.setPower(Range.clip(FRpow, -1, 1));
        motorBR.setPower(Range.clip(BRpow, -1, 1));
    }

    @Deprecated
    public void strafeAssisted(boolean isLeft, double power, double stopRangeCM) { //pass true to strafe left, false to strafe right
        power = Math.abs(power);
        double desiredAngle = imu.getYaw();
        if (isLeft) {
            while (getRightDistance() < stopRangeCM) {
                double diffFromDesired = imu.getTrueDiff(desiredAngle);
                double kP = 0.02; //.025 < PID <.03
                // While this range does work on the trollbot, it has not been tested on the actual robot.
                double PIDchange;

                PIDchange = kP * diffFromDesired;

                setMotors((-power - PIDchange), (power - PIDchange), (power + PIDchange), (-power + PIDchange));
            }
        } else {
            while (getLeftDistance() < stopRangeCM) {
                double diffFromDesired = imu.getTrueDiff(desiredAngle);
                double kP = 0.02; //.025 < PID <.03
                double PIDchange;

                PIDchange = kP * diffFromDesired;

                setMotors((power - PIDchange), (-power - PIDchange), (-power + PIDchange), (power + PIDchange));
            }
        }
    }

    public void strafeRedAssisted(double power, double stopRangeCM, double angle) { //pass true to strafe left, false to strafe right
        power = Math.abs(power);
        //double desiredAngle = angle;


        /*if (isLeft) {
            while (getRightDistance() < stopRangeCM && opModeIsActive()) {
                double diffFromDesired = imu.getTrueDiff(desiredAngle);
                double kP = 0.02; //.025 < PID <.03
                // While this range does work on the trollbot, it has not been tested on the actual robot.
                double PIDchange;

                PIDchange = kP * diffFromDesired;

                setMotors(-power - PIDchange, power - PIDchange, power + PIDchange, -power + PIDchange);
            }
        }
        else {
            while (getRightDistance() > stopRangeCM && opModeIsActive()) {
                double diffFromDesired = imu.getTrueDiff(desiredAngle);
                double kP = 0.02; //.025 < PID <.03
                double PIDchange;

                PIDchange = kP * diffFromDesired;

                setMotors(power - PIDchange, -power - PIDchange, -power + PIDchange, power + PIDchange);
            }
        } */

        if (getRightDistance() > stopRangeCM) {
            while (getRightDistance() > stopRangeCM && opModeIsActive()) {
                strafeRight(power, angle);
            }
            stopMotors();
        } else if (getRightDistance() < stopRangeCM) {
            while (getRightDistance() < stopRangeCM && opModeIsActive()) {
                strafeLeft(power, angle);
            }
            stopMotors();
        }
    }

    public void strafeRedAssistedPID(double targetRange, double angle) {
        double kP = .05;
        double PIDchange;

        times.reset();
        while (Math.abs(getRightDistance() - targetRange) > .5 && opModeIsActive() && times.seconds() < 4) {
            PIDchange = Range.clip(kP * (getRightDistance() - targetRange), -.6, .6);
            if (PIDchange > 0)
                PIDchange = PIDchange < .38 ? .38 : PIDchange;
            else if (PIDchange < 0)
                PIDchange = PIDchange > -.38 ? -.38 : PIDchange;
            strafeRight(PIDchange, angle);
        }
        stopMotors();
    }

    public void strafeBlueAssistedPID(double targetRange, double angle) {
        double kP = .05;
        double PIDchange;

        times.reset();
        while (Math.abs(getLeftDistance() - targetRange) > .5 && opModeIsActive() && times.seconds() < 4) {
            PIDchange = Range.clip(kP * (getLeftDistance() - targetRange), -.6, .6);
            if (PIDchange > 0)
                PIDchange = PIDchange < .35 ? .35 : PIDchange;
            else if (PIDchange < 0)
                PIDchange = PIDchange > -.35 ? -.35 : PIDchange;
            strafeLeft(PIDchange, angle);
        }
        stopMotors();
    }

    public void strafeBlueAssisted(double power, double stopRangeCM, double angle) { //pass true to strafe left, false to strafe right
        power = Math.abs(power);
        double desiredAngle = angle;

        if (getLeftDistance() < stopRangeCM) {
            while (getLeftDistance() < stopRangeCM && opModeIsActive()) {
                strafeRight(power, angle);
            }
            stopMotors();
        } else if (getLeftDistance() > stopRangeCM) {
            while (getLeftDistance() > stopRangeCM && opModeIsActive()) {
                strafeLeft(power, angle);
            }
            stopMotors();
        }
    }

    public void strafeRight(double power, double angle) {
        double desiredAngle = angle;

        double diffFromDesired = imu.getTrueDiff(desiredAngle);
        double kP = 0.01; //.025 < PID <.03
        double PIDchange;

        PIDchange = kP * diffFromDesired;

        setMotors(power - PIDchange, -power - PIDchange, -power + PIDchange, power + PIDchange);
    }

    public void strafeLeft(double power, double angle) {
        double desiredAngle = angle;

        double diffFromDesired = imu.getTrueDiff(desiredAngle);
        double kP = 0.01; //.025 < PID <.03
        double PIDchange;

        PIDchange = kP * diffFromDesired;

        setMotors(-power - PIDchange, power - PIDchange, power + PIDchange, -power + PIDchange);
    }


    public void straightAssisted(double squares) throws InterruptedException {
        straightAssisted(squares, imu.getYaw());
    }

    public void straightAssisted(double squares, double angle) throws InterruptedException {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setTargetPosition((int) -squares * squaresToEncoder);
        motorFL.setTargetPosition((int) squares * squaresToEncoder);
        motorBR.setTargetPosition((int) -squares * squaresToEncoder);
        motorBL.setTargetPosition((int) squares * squaresToEncoder);
    }

    public void Pturn(double angle) throws InterruptedException {
        double kP = .6 / 90;
        double PIDchange;
        double angleDiff = imu.getTrueDiff(angle);
        times.reset();
        while (Math.abs(angleDiff) > 0.5 && opModeIsActive() && times.seconds() < 1.75) {
            angleDiff = imu.getTrueDiff(angle);
            PIDchange = angleDiff * kP;

            if (PIDchange < 0) {
                motorFR.setPower(Range.clip(PIDchange - .1, -1, 1));
                motorBR.setPower(Range.clip(PIDchange - .1, -1, 1));
                motorFL.setPower(Range.clip(-PIDchange + .1, -1, 1));
                motorBL.setPower(Range.clip(-PIDchange + .1, -1, 1));
            } else {
                motorFR.setPower(Range.clip(PIDchange + .1, -1, 1));
                motorBR.setPower(Range.clip(PIDchange + .1, -1, 1));
                motorFL.setPower(Range.clip(-PIDchange - .1, -1, 1));
                motorBL.setPower(Range.clip(-PIDchange - .1, -1, 1));
            }
        }
        stopMotors();
    }

    public double getDist(String color) {
        if (color.equals("RED")) {
            return getRightDistance();
        }
        return getLeftDistance();
    }

    public void knockBall(String color) throws InterruptedException {
        servoUpDownArm.setPosition(.11);
        Thread.sleep(160);
        servoLeftRightArm.setPosition(.24);

        Thread.sleep(500);

        if (jewelIsRed && color.equals("RED")) {
            servoLeftRightArm.setPosition(0);
        } else if (!jewelIsRed && color.equals("RED")) {
            servoLeftRightArm.setPosition(.45);
        } else if (jewelIsRed && color.equals("BLUE")) {
            servoLeftRightArm.setPosition(.45);
        } else if (!jewelIsRed && color.equals("BLUE")) {
            servoLeftRightArm.setPosition(0);
        }

        Thread.sleep(1000);
        servoLeftRightArm.setPosition(.3);
        servoUpDownArm.setPosition(.75);
    }

    public void knockWrongBall(String color) throws InterruptedException {
        servoUpDownArm.setPosition(.11);
        servoLeftRightArm.setPosition(.27);

        Thread.sleep(1000);

        if (jewelIsRed && color.equals("RED")) {
            servoLeftRightArm.setPosition(.45);
        } else if (!jewelIsRed && color.equals("RED")) {
            servoLeftRightArm.setPosition(0);
        }
        if (jewelIsRed && color.equals("BLUE")) {
            servoLeftRightArm.setPosition(0);
        } else if (!jewelIsRed && color.equals("BLUE")) {
            servoLeftRightArm.setPosition(.45);
        }

        Thread.sleep(1000);
        servoLeftRightArm.setPosition(.3);
        servoUpDownArm.setPosition(.55);
    }

    public void grabBlock() throws InterruptedException {

        servoLLHug.setPosition(LLClose);
        servoLRHug.setPosition(LRClose);

        // direction is either 1 or -1, -1 on red 1 on blue
        /*servoLLHug.setPosition(.4);
        servoLRHug.setPosition(.6);
        //moveSquares(-.009, .25);
        startMotors(-.25);
        Thread.sleep(80);
        stopMotors();
        Thread.sleep(200);

        motorYLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorXLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (motorYLift.getCurrentPosition() > -300 && opModeIsActive()) {
            motorYLift.setPower(-.8);
        }
        motorYLift.setPower(0);
        Thread.sleep(201);

        times.reset();
        while (times.milliseconds() < 140 && opModeIsActive()) {
            motorXLift.setPower(-.75);
        }
        motorXLift.setPower(0);
        Thread.sleep(100);

        while (motorYLift.getCurrentPosition() < 300 && opModeIsActive()) {
            motorYLift.setPower(.8);
        }
        motorYLift.setPower(0);
        Thread.sleep(200);*/


        Thread.sleep(500);

        /*times.reset();
        while (times.milliseconds() < 120 && opModeIsActive()) {
            motorXLift.setPower(-.75);
        }
        motorXLift.setPower(0);
        sleep(100); */

        //lift block off ground
        times.reset();
        while (times.milliseconds() < 300 && opModeIsActive()) { //increase this value maybe (500 originally, and used to be based on time)
            motorLiftL.setPower(1);
            motorLiftR.setPower(-1);
        }
        motorLiftL.setPower(0);
        motorLiftR.setPower(0);
    }

    public void liftDown() {
        times.reset();
        while (times.milliseconds() < 120 && opModeIsActive()) { //match value with value of grab block method
            motorLiftL.setPower(-1);
            motorLiftR.setPower(1);
        }
        motorLiftL.setPower(0);
        motorLiftR.setPower(0);
    }

    public void pivot(double desiredAngle) {
        double currAngle = imu.getYaw();
        if (currAngle + 5 < desiredAngle) {
            setMotors(0, 0, .5, .5);
            sleep(300);
        } else if (currAngle - 5 > desiredAngle) {

        }
    }

    public void backUp() {
        setMotors(-.4, -.4, -.4, -.4);
        sleep(100);
        stopMotors();
    }

    public void flip() {
        servoFlip.setPosition(.780);
    }

    public void unflip() {
        servoFlip.setPosition(.025);
    }

    /*public boolean isButtonPressed() {
        return button.getVersion() == 1;
    }*/

    public void getSecondBlockRed() {
        /*
        //1. drive up to pile with paddles open
        servoURHug.setPosition((URClose + UROpen) / 2);
        servoULHug.setPosition((ULClose + ULOpen) / 2);
        //servoLRHug.setPosition((LRClose + LROpen) / 2);
        //servoLLHug.setPosition((LLClose + LLOpen) / 2);
        sleep(50);

        setMotors(.5, .5, .5, .5);
        sleep(250);
        stopMotors();
        sleep(50); */

        //1b. strafe right
        strafeRight(.5, -90);
        sleep(500);
        stopMotors();

        telemetry.addLine("Done strafing");
        telemetry.update();
        sleep(500);

       /* //2. open left, close right (flipped)
        servoURHug.setPosition(UROpen);
        servoULHug.setPosition(ULClose);
        //servoLRHug.setPosition(LRClose);
        //servoLLHug.setPosition(LLOpen);
        sleep(100);

        telemetry.addLine("Left is open");
        telemetry.update();
        sleep(500);*/

        //3. drive forward into block
        setMotors(.3, .3, .3, .3);
        sleep(500);
        stopMotors();
        sleep(50);

        telemetry.addLine("Done driving");
        telemetry.update();
        sleep(500);

        //4. strafe left and close left paddle 45 degrees
        strafeLeft(.5, -90); //
        sleep(500);
        stopMotors();
        servoURHug.setPosition((URClose + UROpen) / 2);
        //servoLLHug.setPosition(.45);
        sleep(50);

        telemetry.addLine("Done strafing left and closing");
        telemetry.update();
        sleep(500);

        //5. strafe slowly and close left paddle slowly at same time
        strafeRight(.5, -90);
        sleep(400);
        /*while (servoURHug.getPosition() < URClose) {
            servoURHug.setPosition(servoURHug.getPosition() + .05);
            //servoLLHug.setPosition(servoLLHug.getPosition() + .5);
            sleep(50);
        } */
        stopMotors();

        servoURHug.setPosition(URClose);
        servoULHug.setPosition(ULClose);

        telemetry.addLine("Done strafing right and closing at same time");
        telemetry.update();
        sleep(500);
    }
}