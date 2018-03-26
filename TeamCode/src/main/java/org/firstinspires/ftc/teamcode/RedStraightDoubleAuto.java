package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.ar.pl.DebugLog;

/**
 * Created by Hamza Ali on 2/5/2018.
 */

@Autonomous(name = "RedStraightDoubleAuto", group = "autonomous")

public class RedStraightDoubleAuto extends CustomLinearOpMode {
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

        times.reset();
        while (times.milliseconds() < 500) {
            strafeLeft(.5, 0);
        }
        stopMotors();
        sleep(300);
        strafeRedAssistedPID(10, 0);
        Pturn(-135);

        liftDown();
        servoULHug.setPosition((ULClose + ULOpen) / 2);
        servoURHug.setPosition((URClose + UROpen) / 2);

        moveSquares(.3, .4);

        servoULHug.setPosition(ULClose);
        servoURHug.setPosition(URClose);
        sleep(500);
        times.reset();
        while (times.milliseconds() < 300 && opModeIsActive()) { //increase this value maybe (500 originally, and used to be based on time)
            motorLiftL.setPower(1);
            motorLiftR.setPower(-1);
        }
        motorLiftL.setPower(0);
        motorLiftR.setPower(0);

        moveSquares(-.3, .3);
        Pturn(0);
        sleep(200);
        moveSquares(.35, .2);

        stopMotors();
        Thread.sleep(250);


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
        Thread.sleep(250);

        servoLLHug.setPosition(.4);
        servoLRHug.setPosition(.6);

        wiggle(.4, 0);
        stopMotors();
        sleep(250);
        backUp();



    }
}
