package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.ar.pl.DebugLog;

/**
 * Created by Hamza Ali on 2/5/2018.
 */

@Autonomous(name = "BlueStraightDoubleAuto", group = "autonomous")
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

        //second block; path to get to the ball will most probably have to be adjusted.
        Pturn(90);
        if (template == 'L')
            Pturn(45);
        else if (template == 'C')
            Pturn(65);
        else if (template == 'R')
            Pturn(85);
        stopMotors();
        Thread.sleep(250);

        moveSquares(1.15,1); //test distance
        Thread.sleep(250);

        grabBlock(); //we'll want to make sure this method actually lifts the block high enough to stack.
        Thread.sleep(250);

        moveSquares(-1.1, 1); //test distance
        Thread.sleep(250);

        Pturn(-90);
        if (template == 'L')
            Pturn(-45);
        else if (template == 'C')
            Pturn(-65);
        else if (template == 'R')
            Pturn(-85);
        stopMotors();

        Thread.sleep(250);

        if (template == 'L') {
            //strafe left
            strafeBlueAssistedPID(47.0, 180);

        } else if (template == 'C') {
            // align with center column
            strafeBlueAssistedPID(63.7, 180);

        } else if (template == 'R') {
            //strafe right
            strafeBlueAssistedPID(81.7, 180);
        } stopMotors();

        Thread.sleep(500);

<<<<<<< HEAD
        wiggle(.4, 0);
        stopMotors();
        servoLHug.setPosition(leftThreadPos);
        servoRHug.setPosition(rightThreadPos);
=======

        servoLLHug.setPosition(.4);
        servoLRHug.setPosition(.6);

        wiggle(.4, 180);

        stopMotors();
        servoLLHug.setPosition(LLThread);
        servoLRHug.setPosition(LRThread);
>>>>>>> 09ebbd6be55045bdce1692388b1cac750036b0ad
        sleep(250);

        backUp();
        liftDown();
<<<<<<< HEAD
        servoLHug.setPosition(.4);
        servoRHug.setPosition(.6);
=======
        servoLLHug.setPosition(.4);
        servoLRHug.setPosition(.6);
>>>>>>> 09ebbd6be55045bdce1692388b1cac750036b0ad
    }
}
