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
        //method grabs block
        strafeLeftDiagonal();

        /*servoULHug.setPosition(ULClose);
        times.reset();
        while (times.milliseconds() < 200) {
            strafeRight(1, -90);
        }
        stopMotors();
        servoURHug.setPosition(URClose);*/

        /*
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
        */

        moveSquares(-.15, .4);

        stopMotors();
        Thread.sleep(100);

        times.reset();
        while (times.milliseconds() < 400 && opModeIsActive()) { //increase this value maybe (500 originally, and used to be based on time)
            motorLiftL.setPower(1);
            motorLiftR.setPower(-1);
        }
        motorLiftL.setPower(0);
        motorLiftR.setPower(0);

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
    }
}
