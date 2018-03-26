package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.ar.pl.DebugLog;

/**
 * Created by hannahbransteter on 10/25/17.
 */

@Autonomous(name = "BStraightWrongBall", group = "autonomous")
public class BStraightWrongBall extends CustomLinearOpMode {
    public void runOpMode() throws InterruptedException {
        initStuff(hardwareMap);

        AutoColor = "BLUE";
        waitForStart();

        getVuMark();
        getJewelColor();

        knockWrongBall(AutoColor);
        Thread.sleep(200);

        grabBlock();
        Thread.sleep(200);

        moveSquares(-.4, .20);
        stopMotors();
        Thread.sleep(500);


        Pturn(-90);
        Pturn(180);
        moveTime(200, -.30);
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
    }
}
