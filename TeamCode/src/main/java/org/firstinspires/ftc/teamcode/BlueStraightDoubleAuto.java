package org.firstinspires.ftc.teamcode;

import com.vuforia.ar.pl.DebugLog;

/**
 * Created by Hamza Ali on 2/5/2018.
 */

public class BlueStraightDoubleAuto extends CustomLinearOpMode {
    public void runOpMode() throws InterruptedException {
        initStuff(hardwareMap);

        AutoColor = "BLUE";
        waitForStart();

        getJewelColor();
        getVuMark();

        knockBall(AutoColor);
        Thread.sleep(200);

        grabBlock();
        Thread.sleep(200);

        moveSquares(-.4, .20);
        stopMotors();
        Thread.sleep(500);


        Pturn(-90);
        Pturn(180);
        stopMotors();
        Thread.sleep(500);

        DebugLog.LOGE("startDistance ", "" + getLeftDistance());



        if (template == 'L') {
            //strafe left
            strafeBlueAssistedPID(47.0, 180);

        } else if (template == 'C') {
            // align with center column
            strafeBlueAssistedPID(63.7, 180);

        } else if (template == 'R') {
            //strafe right
            //strafeBlueAssisted(.5, 77, 180);
            strafeBlueAssistedPID(81.7, 180);
        } stopMotors();

        liftDown();
        Thread.sleep(500);

        servoLLHug.setPosition(.4);
        servoLRHug.setPosition(.6);

        wiggle(.4, 180);
        stopMotors();
        sleep(250);
        backUp();

        //second block
        Pturn(90);
        Pturn(45);
        Thread.sleep(250);

        moveSquares(1.45,1);
        Thread.sleep(250);

        grabBlock(); //we'll want to make sure this method actually lifts the block high enough to stack.
        Pturn(-90);
        Pturn(-90);

        moveSquares(1.45, 1);
        Thread.sleep(250);

        Pturn(45);
        stopMotors();
        Thread.sleep(500);

        if (template == 'L') {
            //strafe left
            strafeBlueAssistedPID(47.0, 180);

        } else if (template == 'C') {
            // align with center column
            strafeBlueAssistedPID(63.7, 180);

        } else if (template == 'R') {
            //strafe right
            //strafeBlueAssisted(.5, 77, 180);
            strafeBlueAssistedPID(81.7, 180);
        } stopMotors();

        liftDown();
        Thread.sleep(500);

        servoLLHug.setPosition(.4);
        servoLRHug.setPosition(.6);

        wiggle(.4, 180);
        stopMotors();
        sleep(250);
        backUp();
    }
}
