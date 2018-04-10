package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.ar.pl.DebugLog;

/**
 * Created by Ryan Branstetter on 10/4/17.
 */

@Autonomous(name = "BlueSideDoubleAuto", group = "autonomous")
public class BlueSideDoubleAuto extends CustomLinearOpMode {
    public void runOpMode() throws InterruptedException {
        initStuff(hardwareMap);

        AutoColor = "BLUE";
        waitForStart();

        getVuMark();
        getJewelColor();

        knockBall(AutoColor);
        Thread.sleep(200);

        grabBlock();

        flip();
        Thread.sleep(200);

        moveSquares(-.65, .20);
        stopMotors();

        Thread.sleep(100);

        //back up slightly
        moveSquares(.10, .20);
        stopMotors();
        Thread.sleep(200);

        //turn towards glyph pit
        Pturn(-90);
        liftDown();
        servoULHug.setPosition(ULOpen);
        servoURHug.setPosition(URThread);

        moveSquares(.44, .4);

        blueSecondBlock();

        liftUp();

        unflip();
        Pturn(90);

        DebugLog.LOGE("startDistance ", "" + getLeftDistance());

        if (template == 'L') {
            //strafe left
            strafeBlueAssistedPID(22, 90);
            DebugLog.LOGE("VuMark: ", "L");

        } else if (template == 'C') {
            // align with center column
            strafeBlueAssistedPID(40, 90);
            DebugLog.LOGE("VuMark: ", "C");

        } else if (template == 'R') {
            //strafe right
            strafeBlueAssistedPID(55.0, 90);
            DebugLog.LOGE("VuMark: ", "R");
        } stopMotors();

        DebugLog.LOGE("End Distance: ", "" + getLeftDistance());

        liftDown();
        Thread.sleep(400);

        servoLLHug.setPosition((LLClose + LLOpen) / 2);
        servoLRHug.setPosition((LRClose + LROpen) / 2);
        servoULHug.setPosition((ULClose + ULOpen) / 2);
        servoURHug.setPosition((URClose + UROpen) / 2);

        wiggle(.4, 90);
        stopMotors();

        backUp();

        /*//third block???
        Pturn(-90);
        servoLLHug.setPosition(LLOpen);
        servoLRHug.setPosition(LRThread);
        servoULHug.setPosition(ULOpen);
        servoURHug.setPosition(URThread);

        moveSquares(.4, .4);
        thirdBlockRed();

        Pturn(90);

        DebugLog.LOGE("startDistance ", "" + getLeftDistance());

        if (template == 'L') {
            //strafe left
            strafeBlueAssistedPID(22, 90);
            DebugLog.LOGE("VuMark: ", "L");

        } else if (template == 'C') {
            // align with center column
            strafeBlueAssistedPID(40, 90);
            DebugLog.LOGE("VuMark: ", "C");

        } else if (template == 'R') {
            //strafe right
            strafeBlueAssistedPID(55.0, 90);
            DebugLog.LOGE("VuMark: ", "R");
        } stopMotors();

        DebugLog.LOGE("End Distance: ", "" + getLeftDistance());

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
