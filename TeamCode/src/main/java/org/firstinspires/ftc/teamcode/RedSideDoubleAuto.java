package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.ar.pl.DebugLog;

/**
 * Created by Ryan Branstetter on 10/4/17.
 */

@Autonomous(name = "RedSideDoubleAuto", group = "autonomous")
public class RedSideDoubleAuto extends CustomLinearOpMode {
    public void runOpMode() throws InterruptedException {
        initStuff(hardwareMap);

        AutoColor = "RED";
        waitForStart();

        getVuMark();
        getJewelColor();

        knockBall(AutoColor);
        Thread.sleep(200);

        grabBlock();

        flip();
        Thread.sleep(200);

        moveSquares(.65, .20);
        stopMotors();
        Thread.sleep(100);

        Pturn(-90);
        liftDown();
        servoULHug.setPosition(ULThread);
        servoURHug.setPosition(UROpen);
        moveSquares(.44, .4);
        strafeRightDiagonal();

        //servoULHug.setPosition(ULClose);
        //times.reset();
        //while (times.milliseconds() < 200) {
        //    strafeRight(1, -90);
        //}
        //stopMotors();
        //servoURHug.setPosition(URClose);

        /*servoULHug.setPosition(ULClose);
        servoURHug.setPosition(URClose);
        sleep(500);
        times.reset();
        while (times.milliseconds() < 300 && opModeIsActive()) { //increase this value maybe (500 originally, and used to be based on time)
            motorLiftL.setPower(1);
            motorLiftR.setPower(-1);
        }
        motorLiftL.setPower(0);
        motorLiftR.setPower(0);

        */

        //getSecondBlockRed();


        moveSquares(-.28, .4);
        stopMotors();
        Thread.sleep(100);

        times.reset();
        while (times.milliseconds() < 400 && opModeIsActive()) { //increase this value maybe (500 originally, and used to be based on time)
            motorLiftL.setPower(1);
            motorLiftR.setPower(-1);
        }
        motorLiftL.setPower(0);
        motorLiftR.setPower(0);

        unflip();
        Pturn(90);


        DebugLog.LOGE("startDistance ", "" + getRightDistance());

        boolean side = true;
        if (template == 'L') {
            //strafe left
            strafeRedAssistedPID(55.4, 90);
            DebugLog.LOGE("Template: ", "L");

        } else if (template == 'C') {
            // align with center column
            strafeRedAssistedPID( 36.7, 90);
            DebugLog.LOGE("Template: ", "C");

        } else if (template == 'R') {
            //strafe right
            strafeRedAssistedPID(21.7, 90);
            DebugLog.LOGE("Template: ", "R");
        } stopMotors();

        liftDown();
        Thread.sleep(400);

        servoLLHug.setPosition((LLClose + LLOpen) / 2);
        servoLRHug.setPosition((LRClose + LROpen) / 2);
        servoULHug.setPosition((ULClose + ULOpen) / 2);
        servoURHug.setPosition((URClose + UROpen) / 2);

        wiggle(.4, 90);
        stopMotors();
        backUp();

        //Pturn(-90);
        //Pturn(-90);

        //getSecondBlockRed();

        //Pturn(90);
        //Pturn(90);
    }
}