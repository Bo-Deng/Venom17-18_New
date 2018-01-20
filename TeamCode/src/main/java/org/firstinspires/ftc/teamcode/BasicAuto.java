package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.ar.pl.DebugLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Ryan Bransteter on 9/30/17.
 */


@Autonomous(name = "BasicAuto", group = "autonomous")
public class BasicAuto extends LinearOpMode {
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;
    ModernRoboticsI2cRangeSensor leftRangeSensor;
    ModernRoboticsI2cRangeSensor rightRangeSensor;
    // Servo rightWallServo;

    String AutoColor;

    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

public void runOpMode() throws InterruptedException {
    motorFL = hardwareMap.dcMotor.get("motorFL");
    motorFR = hardwareMap.dcMotor.get("motorFR");
    motorBL = hardwareMap.dcMotor.get("motorBL");
    motorBR = hardwareMap.dcMotor.get("motorBR");
    motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
    motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
    leftRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeL");
    rightRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeR");
    // rightWallServo = hardwareMap.get("RightWallServo");

    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
    parameters.vuforiaLicenseKey = "AXb/g5n/////AAAAGSUed2rh5Us1jESA1cUn5r5KDUqTfwO2woh7MxjiLKSUyDslqBAgwCi0Qmc6lVczErnF5TIw7vG5R4TJ2igvrDVp+dP+3i2o7UUCRRj/PtyVgb4ZfNrDzHE80/6TUHifpKu4QCM04eRWYZocWNWhuRfytVeWy6NSTWefM9xadqG8FFrFk3XnvqDvk/6ZAgerNBdq5SsJ90eDdoAhgYEee40WxasoUUM9YVMvkWOqZgHSuraV2IyIUjkW/u0O+EkFtTNRUWP+aZwn1qO1H4Lk07AJYe21eqioBLMdzY7A8YqR1TeQ//0WJg8SFdXjuGbF6uHykBe2FF5UeyaehA0iTqfPS+59FLm8y1TuUt57eImq";
    parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

    VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
    VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        relicTrackables.activate();


    // copy pasta from the ftc ppl
    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
    char template = ' ';

        telemetry.addData("VuMark ", vuMark);
        while (vuMark == RelicRecoveryVuMark.UNKNOWN) {
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
    }

        telemetry.addData("VuMark ", vuMark);
        if (vuMark == RelicRecoveryVuMark.CENTER)
    template = 'C';
        else if (vuMark == RelicRecoveryVuMark.LEFT)
    template = 'L';
        else if (vuMark == RelicRecoveryVuMark.RIGHT)
    template = 'R';

        telemetry.update();

    waitForStart();

    /*
    // testing jewels, psuedo code
    // if always detects on left jewel

    // rightWallServo.setPosition(.25);

    if (getColor().equals(AutoColor)) {
        startMotors(-.10, -.10);
        Thread.sleep(100); }
    else if (!getColor.equals(AutoColor) && !getColor.equals("GREEN")){
        startMotors(.10, .10);
        Thread.sleep(100); }
    else {
        wallServo.setPosition(0); }

    // wallServo.setPosition(0);
    */



    //Forward till close to glyph container. (using range sensor??)

    startMotors(.20, .20);
    Thread.sleep(2 * 1000);

    stopMotors();

    //Turn towards glyph container.



    //Align with correct column.

        DebugLog.LOGE("startDistance ", "" + getRightDistance());

        if (template == 'L') {
        //strafe left
        strafe(0, 1);
        while (getRightDistance() < 100) {
        }
        stopMotors();
    }

        else if (template == 'C') {
        // align with center column
        strafe(0, 1);
        while (getRightDistance() < 80) {
        }
        stopMotors();
    }

        else if (template == 'R') {
        //strafe right
        strafe(0, 1);
        while (getRightDistance() < 60) {
        }
        stopMotors();
    }

    // Place glyph into column and park.

    startMotors(.10, .10);
        Thread.sleep(1000);

        DebugLog.LOGE("at end ", "ended");
    stopMotors();
}

    public void stopMotors() {
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    public void startMotors(double rSpeed, double lSpeed) {
        motorFL.setPower(lSpeed);
        motorFR.setPower(rSpeed);
        motorBL.setPower(lSpeed);
        motorBR.setPower(rSpeed);
    }

    public void turn(double speed, int time) throws InterruptedException {
        startMotors(speed, -speed);
        Thread.sleep(time);
        stopMotors();
    }

    public void strafe(int d, int p) throws InterruptedException { // d = direction, p = power, t = time
        if (d == 0) { //left
            motorFL.setPower(p);
            motorFR.setPower(-p);
            motorBL.setPower(-p);
            motorBR.setPower(p);
        } else if (d == 1) { //right
            motorFL.setPower(-p);
            motorFR.setPower(p);
            motorBL.setPower(p);
            motorBR.setPower(-p);
        }
    }
    public double getRightDistance() {
        double dist = rightRangeSensor.getDistance(DistanceUnit.CM);
        while (dist > 1000) {
            dist = rightRangeSensor.getDistance(DistanceUnit.CM);
        }
        return dist;
    }
}
