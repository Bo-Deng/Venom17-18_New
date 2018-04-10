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
        Thread.sleep(200);

       //back up slightly
        moveSquares(-.10, .20);
        stopMotors();
        Thread.sleep(200);

        //turn towards glyph pit
        Pturn(-90);
        liftDown();
        servoULHug.setPosition(ULThread);
        servoURHug.setPosition(UROpen);
        moveSquares(.44, .4);

        redSecondBlock();

        liftUp();

        unflip();
        Pturn(90);

        DebugLog.LOGE("startDistance ", "" + getRightDistance());

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
        Thread.sleep(200);

        servoLLHug.setPosition((LLClose + LLOpen) / 2);
        servoLRHug.setPosition((LRClose + LROpen) / 2);
        servoULHug.setPosition((ULClose + ULOpen) / 2);
        servoURHug.setPosition((URClose + UROpen) / 2);

        wiggle(.4, 90);
        stopMotors();
        backUp();

        /* //third block???
        Pturn(-90);
        servoLLHug.setPosition(LLThread);
        servoLRHug.setPosition(LROpen);
        servoULHug.setPosition(ULThread);
        servoURHug.setPosition(UROpen);

        moveSquares(.4, .4);
        thirdBlockRed();

        Pturn(90);

        DebugLog.LOGE("startDistance ", "" + getRightDistance());

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
        Thread.sleep(200);

        servoLLHug.setPosition((LLClose + LLOpen) / 2);
        servoLRHug.setPosition((LRClose + LROpen) / 2);
        servoULHug.setPosition((ULClose + ULOpen) / 2);
        servoURHug.setPosition((URClose + UROpen) / 2);

        wiggle(.4, 90);
        stopMotors();
        backUp(); */
    }
}