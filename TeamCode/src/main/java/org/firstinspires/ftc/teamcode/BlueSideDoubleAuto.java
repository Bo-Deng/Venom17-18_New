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

        getJewelColor();
        getVuMark();

        knockBall(AutoColor);
        Thread.sleep(200);

        grabBlock();
        Thread.sleep(200);

        moveSquares(-.85, .20);
        stopMotors();

        Thread.sleep(500);

        Pturn(90);
        Thread.sleep(500);

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

        servoLHug.setPosition(.4);
        servoRHug.setPosition(.6);

        wiggle(.4, 90);
        stopMotors();
        sleep(200);
        backUp();
        //moveSquares(-.2, .4);

        //second block, test please
        Pturn(-90);
        Pturn(-90);
        moveSquares(.9, 1);
        stopMotors();
        Thread.sleep(200);
        grabBlock(); //we'll want to make sure this method actually lifts the block high enough to stack.
        Pturn(90);
        Pturn(90);

        Thread.sleep(500);
        moveSquares(.92, 1);

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

        Thread.sleep(200);

        wiggle(.4, 90);
        stopMotors();

        servoLHug.setPosition(leftThreadPos);
        servoRHug.setPosition(rightThreadPos);

        sleep(200);
        backUp();
        stopMotors();

        liftDown();
        servoLHug.setPosition(.4);
        servoRHug.setPosition(.6);
    }
}
