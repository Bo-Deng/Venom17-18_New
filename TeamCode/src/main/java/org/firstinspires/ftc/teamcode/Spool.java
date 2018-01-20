package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by hannahbransteter on 1/9/18.
 */


@TeleOp(name="Spool", group="opMode")
public class Spool extends CustomOpMode {


    public void init() {

        initStuff(hardwareMap);


        telemetry.addData("init ", "completed");
        telemetry.update();
    }

    @Override
    public void loop() {
       if (Math.abs(gamepad2.left_stick_y) > .1) {
           motorRelicTop.setPower(gamepad2.left_stick_y);
       } else {
           motorRelicTop.setPower(0);
       }
       if (Math.abs(gamepad2.right_stick_y) > .1) {
           //motorRelicBottom.setPower(gamepad2.right_stick_y);
       } else {
           //motorRelicBottom.setPower(0);
       }
    }
}
