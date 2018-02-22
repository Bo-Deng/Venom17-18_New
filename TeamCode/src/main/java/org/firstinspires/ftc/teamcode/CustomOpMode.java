package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import for_camera_opmodes.OpModeCamera;

/**
 * Created by Bo on 9/25/2017.
 */
public class CustomOpMode extends OpMode {

    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;

    Servo servoLLHug;
    Servo servoLRHug;
    Servo servoULHug;
    Servo servoURHug;
    Servo servoFlip;

    Servo servoUpDownArm;
    Servo servoLeftRightArm;

    IMU imu;

    int squaresToEncoder = 1120; //use motorBL

    ElapsedTime time;

    ModernRoboticsI2cRangeSensor rangeSensorL;
    ModernRoboticsI2cRangeSensor rangeSensorR;

    DcMotor motorLiftL;
    DcMotor motorLiftR;

    DcMotor motorRelicTop;
    DcMotor motorRelicBottom;

    Servo servoRelicRot;
    Servo servoRelicGrab;

    String AutoColor;

    double sf = 1.3;
    double startVoltage;
    boolean buttonEnabled = true;

    //left hug variables
    /*double leftInitPos = .649;
    double leftOpenPos = .55;
    double leftThreadPos = .32;
    double leftClampPos = .18;
    //right hug variables
    double rightInitPos = .43;
    double rightOpenPos = .49;
    double rightThreadPos = .8;
    double rightClampPos = 1.0;*/

    double grabOpenPos = .925;
    double grabClosePos = .025;
    double rotOpenPos = .8175;
    double rotClosePos = .032;

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

    double c = .25;



    AnalogInput button;

    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    public void init() {

    }

    public void loop() {

    }

    public void initStuff(HardwareMap map) {
        time = new ElapsedTime();

        motorFR = map.dcMotor.get("motorFR");
        motorFL = map.dcMotor.get("motorFL");
        motorBR = map.dcMotor.get("motorBR");
        motorBL = map.dcMotor.get("motorBL");

        servoLLHug = map.servo.get("servoLRHug");
        servoLRHug = map.servo.get("servoLLHug");
        servoULHug = map.servo.get("servoURHug");
        servoURHug = map.servo.get("servoULHug");
        servoFlip = map.servo.get("servoFlip");

        servoLeftRightArm = map.servo.get("servoLeftRightArm");
        servoUpDownArm = map.servo.get("servoUpDownArm");

        motorLiftL = map.dcMotor.get("motorLiftL");
        motorLiftR = map.dcMotor.get("motorLiftR");

        //servoLLHug.setPosition(leftInitPos);
        //servoLRHug.setPosition(rightInitPos);
        servoLLHug.setPosition(LLOpen);
        servoLRHug.setPosition(LROpen);
        servoULHug.setPosition(ULOpen);
        servoURHug.setPosition(UROpen);

        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
        }

        servoFlip.setPosition(.025);

        servoUpDownArm.setPosition(.73);
        servoLeftRightArm.setPosition(.35);

        servoRelicGrab = map.servo.get("servoRelicGrab");
        servoRelicRot = map.servo.get("servoRelicRot");

        servoRelicGrab.setPosition(grabClosePos);
        servoRelicRot.setPosition(0);

        motorRelicTop = map.dcMotor.get("motorRelicTop");
        motorRelicBottom = map.dcMotor.get("motorRelicBottom");

        rangeSensorL = map.get(ModernRoboticsI2cRangeSensor.class, "rangeL");
        rangeSensorR = map.get(ModernRoboticsI2cRangeSensor.class, "rangeR");

        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLiftL.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRelicTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRelicBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLiftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLiftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRelicTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRelicBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        button = hardwareMap.get(AnalogInput.class, "button");
        startVoltage = button.getVoltage();

        imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
        imu.IMUinit(hardwareMap);

        telemetry.addData("init ", "completed");
        telemetry.update();
    }

    public void setMode(DcMotor.RunMode runMode){
        motorFR.setMode(runMode);
        motorFL.setMode(runMode);
        motorBR.setMode(runMode);
        motorBL.setMode(runMode);
    }

    public double getRightDistance() {
        double dist = rangeSensorR.getDistance(DistanceUnit.CM);
        /*while (dist > 1000 || Double.isNaN(dist)) {
            dist = rangeSensorR.getDistance(DistanceUnit.CM);
        }*/
        return dist;
    }

    public double getLeftDistance() {
        double dist = rangeSensorL.getDistance(DistanceUnit.CM);
        /*while (dist > 1000 || Double.isNaN(dist)) {
            dist = rangeSensorL.getDistance(DistanceUnit.CM);
        }*/
        return dist;
    }
    public double rightABSMotorVal(double joyStickVal) {
        /*if (Math.abs(joyStickVal - motorBL.getPower()) < 1) {
            return joyStickVal;
        }*/
        if (-joyStickVal >= motorBR.getPower()) {
            return Range.clip(motorBR.getPower() + c, -1, -joyStickVal);
        }
        else if (-joyStickVal < motorBR.getPower()) {
            return Range.clip(motorBR.getPower() - c, -joyStickVal, 1);
        }
        else return -joyStickVal;
    }
    public double leftABSMotorVal(double joyStickVal) {
        if (-joyStickVal >= motorBL.getPower()) {
            return Range.clip(motorBL.getPower() + c, -1, -joyStickVal);
        }
        else if (-joyStickVal < motorBL.getPower()) {
            return Range.clip(motorBL.getPower() - c, -joyStickVal, 1);
        }
        else return -joyStickVal;
    }


    public void stopMotor() {
        if (motorBL.getPower() > 0) {
            motorBL.setPower(Range.clip(motorBL.getPower() - c, 0, 1));
        }
        else if (motorBL.getPower() < 0) {
            motorBL.setPower(Range.clip(motorBL.getPower() + c, -1, 0));
        }

        if (motorBR.getPower() > 0) {
            motorBR.setPower(Range.clip(motorBR.getPower() - c, 0, 1));
        }
        else if (motorBR.getPower() < 0) {
            motorBR.setPower(Range.clip(motorBR.getPower() + c, -1, 0));
        }

        if (motorFL.getPower() > 0)
            motorFL.setPower(Range.clip(motorFL.getPower() - c, 0, 1));
        else if (motorFL.getPower() < 0)
            motorFL.setPower(Range.clip(motorFL.getPower() + c, -1, 0));

        if (motorFR.getPower() > 0)
            motorFR.setPower(Range.clip(motorFR.getPower() - c, 0, 1));
        else if (motorFR.getPower() < 0)
            motorFR.setPower(Range.clip(motorFR.getPower() + c, -1, 0));
    }

    public boolean signsAreDifferent(double x, double y) {
        if (x > 0)
            return y < 0;
        if (x < 0)
            return y > 0;
        return false;
    }
    public boolean isButtonPressed() {
        return Math.abs(button.getVoltage() - startVoltage) > .2 && buttonEnabled;
    }
}
