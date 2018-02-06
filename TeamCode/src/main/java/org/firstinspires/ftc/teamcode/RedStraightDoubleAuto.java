package org.firstinspires.ftc.teamcode;

import com.vuforia.ar.pl.DebugLog;

/**
 * Created by Hamza Ali on 2/5/2018.
 */

public class RedStraightDoubleAuto extends CustomLinearOpMode {
    public void runOpMode() throws InterruptedException {
        initStuff(hardwareMap);

        AutoColor = "RED";
        waitForStart();

        getJewelColor();
        getVuMark();

        knockBall(AutoColor);
        Thread.sleep(200);

        grabBlock();
        Thread.sleep(200);

        moveSquares(.35, .20);
        stopMotors();
        Thread.sleep(500);


        DebugLog.LOGE("startDistance ", "" + getRightDistance());

        if (template == 'L') {
            //strafe left
            strafeRedAssistedPID(80.4, 0);

        } else if (template == 'C') {
            // align with center column
            strafeRedAssistedPID(62.9, 0);

        } else if (template == 'R') {
            //strafe right
            strafeRedAssistedPID(46, 0);
        } stopMotors();

        liftDown();
        Thread.sleep(500);

        servoLHug.setPosition(.4);
        servoRHug.setPosition(.6);

        wiggle(.4, 0);
        stopMotors();
        sleep(250);
        backUp();

        //second block, path to get to the ball will most probably have to be adjusted.
        Pturn(-90);
        Pturn(-45);
        Thread.sleep(250);

        moveSquares(1.45,1); //test distance
        Thread.sleep(250);

        grabBlock(); //we'll want to make sure this method actually lifts the block high enough to stack.
        Pturn(90);
        Pturn(90);

        moveSquares(1.5, 1); //test distance
        Thread.sleep(250);

        Pturn(-45);
        stopMotors();
        Thread.sleep(500);

        if (template == 'L') {
            //strafe left
            strafeRedAssistedPID(80.4, 0);

        } else if (template == 'C') {
            // align with center column
            strafeRedAssistedPID(62.9, 0);

        } else if (template == 'R') {
            //strafe right
            strafeRedAssistedPID(46, 0);
        } stopMotors();

        liftDown();
        Thread.sleep(500);

        servoLHug.setPosition(.4);
        servoRHug.setPosition(.6);

        wiggle(.4, 0);
        stopMotors();
        sleep(250);
        backUp();
    }
}
