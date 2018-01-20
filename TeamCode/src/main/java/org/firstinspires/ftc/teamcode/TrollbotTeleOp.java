package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Bo on 9/4/2017.
 */
@TeleOp(name="trollbot tele", group="opMode")
public class TrollbotTeleOp extends OpMode {


    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;
    IMU imu;

    double maintainAngle;


    public void init() {

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
        imu.IMUinit(hardwareMap);

        telemetry.addData("init ", "completed");
        telemetry.update();
    }
    @Override
    public void loop() {

        /*// if statement for forward/backward
        if (Math.abs(gamepad1.left_stick_y) > .1 && Math.abs(gamepad1.left_stick_x) < .1) {

            startMotor(gamepad1.left_stick_y);
        }
        // if statement for left/right
        if (Math.abs(gamepad1.left_stick_y) < .1 && Math.abs(gamepad1.left_stick_x) > .1)
        {
            motorFR.setPower(gamepad1.left_stick_x);
            motorBR.setPower(-gamepad1.left_stick_x);
            motorFL.setPower(-gamepad1.left_stick_x);
            motorBL.setPower(gamepad1.left_stick_x);
        }*/
        /*// if (Math.abs(gamepad1.left_stick_y) > .1 || Math.abs(gamepad1.left_stick_x) > .1 || Math.abs(gamepad1.right_stick_x) > .1)
        {
            double x = Math.abs(gamepad1.left_stick_y) > .1 ? gamepad1.left_stick_y : 0;
            double y = Math.abs(gamepad1.left_stick_x) > .1 ? gamepad1.left_stick_x : 0;
            double r = Math.abs(gamepad1.right_stick_x) > .1 ? gamepad1.right_stick_x : 0;
            double max = Math.max(Math.abs(x + y + r), Math.max(Math.abs(x - y - r), Math.max(Math.abs(x - y + r), Math.abs(x + y - r))));
            if (max < 1)
                max = 1;

            motorFL.setPower((x - y - r) / max);
            motorFR.setPower((x + y + r) / max);
            motorBL.setPower((x + y - r) / max);
            motorBR.setPower((x - y + r) / max);
        }
        else
            stopMotor();
*/
        if (gamepad1.a)
            motorBL.setPower(1);
        if (gamepad1.b)
            motorBR.setPower(1);
        if (gamepad1.y)
            motorFL.setPower(1);
        if (gamepad1.x)
            motorFR.setPower(1);


        double yL = gamepad1.left_stick_y;
        double yR = gamepad1.right_stick_y;
        double rt = gamepad1.right_trigger;
        double lt = gamepad1.left_trigger;
        double r = Math.max(Math.abs(rt), Math.abs(lt));
        /*// double max = Math.max(Math.abs(x + y + r), Math.max(Math.abs(x - y - r), Math.max(Math.abs(x - y + r), Math.abs(x + y - r))));
        if (max < 1)
            max = 1;

        motorFL.setPower((x - y - r) / max);
        motorFR.setPower((x + y + r) / max);
        motorBL.setPower((x + y - r) / max);
        motorBR.setPower((x - y + r) / max); */
        if (Math.abs(yL) > .1 || Math.abs(yR) > .1) {
            motorBL.setPower(-yL);
            motorFL.setPower(-yL);
            motorBR.setPower(-yR);
            motorFR.setPower(-yR);
        }
        else if (rt != 0){
            motorBL.setPower(-rt);
            motorFL.setPower(rt);
            motorBR.setPower(rt);
            motorFR.setPower(-rt);
        }
        else if (lt != 0){
            motorBL.setPower(lt);
            motorFL.setPower(-lt);
            motorBR.setPower(-lt);
            motorFR.setPower(lt);
        }
        else
            stopMotor();
    }

    public void startMotor(double Speed) {
        motorFL.setPower(Speed);
        motorBL.setPower(Speed);
        motorFR.setPower(Speed);
        motorBR.setPower(Speed);
    }

    public void stopMotor() {
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }
}
