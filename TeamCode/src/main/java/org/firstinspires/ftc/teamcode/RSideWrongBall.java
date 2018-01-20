package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.ar.pl.DebugLog;

/**
 * Created by hannahbransteter on 10/25/17.
 */

@Autonomous(name = "RSideWrongBall", group = "autonomous")
public class RSideWrongBall extends CustomLinearOpMode {
    public void runOpMode() throws InterruptedException {
        initStuff(hardwareMap);

        AutoColor = "RED";
        waitForStart();
        getJewelColor();
        getVuMark();

        knockWrongBall(AutoColor);
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
            strafeRedAssisted(.5, 50, 90);
            DebugLog.LOGE("Template: ", "L");

        } else if (template == 'C') {
            // align with center column
            strafeRedAssisted(.5, 35, 90);
            DebugLog.LOGE("Template: ", "C");

        } else if (template == 'R') {
            //strafe right
            strafeRedAssistedPID(20.7, 90);
            DebugLog.LOGE("Template: ", "R");
        } stopMotors();

        liftDown();
        Thread.sleep(500);

        servoLHug.setPosition(.4);
        servoRHug.setPosition(.6);

        wiggle(.4, 90);
        stopMotors();
        sleep(250);
        backUp();
    }
}
