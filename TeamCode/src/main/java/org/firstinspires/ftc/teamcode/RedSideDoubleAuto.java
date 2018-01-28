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
        getJewelColor();
        getVuMark();

        knockBall(AutoColor);
        Thread.sleep(200);

        grabBlock();
        Thread.sleep(200);

        moveSquares(.65, .20);
        stopMotors();
        Thread.sleep(500);


        Pturn(90);
        stopMotors();
        Thread.sleep(500);

        DebugLog.LOGE("startDistance ", "" + getRightDistance());

        boolean side = true;
        if (template == 'L') {
            //strafe left
            strafeRedAssistedPID(53.4, 90);
            DebugLog.LOGE("Template: ", "L");

        } else if (template == 'C') {
            // align with center column
            strafeRedAssistedPID( 36.7, 90);
            DebugLog.LOGE("Template: ", "C");

        } else if (template == 'R') {
            //strafe right
            strafeRedAssistedPID(20.7, 90);
            DebugLog.LOGE("Template: ", "R");
        } stopMotors();

        liftDown();
        Thread.sleep(200);

        servoLHug.setPosition(.4);
        servoRHug.setPosition(.6);

        wiggle(.4, 90);
        stopMotors();
        sleep(200);
        backUp();
        sleep(200);
        Pturn(-90);
        moveSquares(1.5, 1);
        grabBlock();
        Pturn(0);
        moveSquares(2.75, 1);


    }
}