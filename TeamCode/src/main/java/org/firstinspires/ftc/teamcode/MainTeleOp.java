package org.firstinspires.ftc.teamcode;

/**
 * Created by Ryan Bransteter on 9/30/17.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Main teleop", group="opMode")
public class MainTeleOp extends CustomOpMode {

    double motorScale = 1;
    boolean isFlipped = false;

    double rollOffset;
    double pitchOffset;

    public void init() {

        initStuff(hardwareMap);
        try {
            Thread.sleep(500);
        }
        catch (Exception e) {

        }
        rollOffset = imu.getRoll();
        pitchOffset = imu.getPitch();

        telemetry.addData("init ", "completed");
        telemetry.update();
    }
    @Override
    public void loop() {
        // for testing hardware mapping
        if (gamepad1.a) {
            motorScale = motorScale == .5 ? 1 : .5;
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
            }
        }

        if (gamepad1.back) {
            buttonEnabled = !buttonEnabled;
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
            }
        }


        /*if (gamepad1.dpad_up) {
            motorBL.setPower(.2);
            motorBR.setPower(.2);
            motorFL.setPower(.2);
            motorFR.setPower(.2);
        }*/

        // the signs might need to be switched
        if (gamepad2.left_stick_y > 0.1) {
            motorRelicTop.setPower(gamepad2.left_stick_y);
        } else if (gamepad2.left_stick_y < -0.1) {
            motorRelicTop.setPower(gamepad2.left_stick_y);
        } else {
            motorRelicTop.setPower(0);
        }


        if (gamepad2.right_stick_y < -0.1) {
            motorLiftL.setPower(gamepad2.right_stick_y);
            motorLiftR.setPower(gamepad2.right_stick_y);

        } else if (gamepad2.right_stick_y > 0.1 /*&& !isButtonPressed()*/) {
            motorLiftL.setPower(gamepad2.right_stick_y / 2);
            motorLiftR.setPower(gamepad2.right_stick_y / 2);

        } /*else if (isButtonPressed()) {
            motorLiftL.setPower(-1);
            motorLiftR.setPower(-1);
        } */else {
            motorLiftL.setPower(0);
            motorLiftR.setPower(0);
        }

        double yL = -gamepad1.left_stick_y;
        double yR = -gamepad1.right_stick_y;
        double rt = gamepad1.right_trigger;
        double lt = gamepad1.left_trigger;


        // forwards and backwards


        if (Math.abs(yL) > .1 || Math.abs(yR) > .1) {

            motorBL.setPower(leftABSMotorVal(-yL) * motorScale);
            motorFL.setPower(leftABSMotorVal(-yL) * motorScale);

            motorBR.setPower(rightABSMotorVal(-yR) * motorScale);
            motorFR.setPower(rightABSMotorVal(-yR) * motorScale);

        }
        // strafe right
        else if (Math.abs(rt) > .1) {
            motorBL.setPower(-rt * motorScale);
            motorFL.setPower(rt * motorScale);
            motorBR.setPower(rt * motorScale);
            motorFR.setPower(-rt * motorScale);
        }
        // strafe left
        else if (Math.abs(lt) > .1) {
            motorBL.setPower(lt * motorScale);
            motorFL.setPower(-lt * motorScale);
            motorBR.setPower(-lt * motorScale);
            motorFR.setPower(lt * motorScale);
        } else if (gamepad1.x) {
            //kP constants for the forwards/backwards and left/right directions
            double kP_FB = .0496;
            double kP_LR = .0754;

            //retrieve error for pitch and roll
            double diffPitch = imu.getPitch() - pitchOffset;
            double diffRoll = imu.getRoll() - rollOffset;

            telemetry.addData("Pitch: ", diffPitch);
            telemetry.addData("Roll: ", diffRoll);

            //calculate action applied
            double PIDchangeFB = -kP_FB * diffRoll;
            double PIDchangeLR = -kP_LR * diffPitch;

            //scales applied power down if greater than 1
            double max = Math.max(Math.abs(PIDchangeFB + PIDchangeLR), Math.abs(PIDchangeFB - PIDchangeLR));
            max = max > 1 ? max : 1;
            motorFL.setPower((PIDchangeFB + PIDchangeLR) / max);
            motorFR.setPower((PIDchangeFB - PIDchangeLR) / max);
            motorBL.setPower((PIDchangeFB - PIDchangeLR) / max);
            motorBR.setPower((PIDchangeFB + PIDchangeLR) / max);
        } else
            stopMotor();

        /*if (gamepad2.dpad_left) {
            //servoLLHug.setPosition(Range.clip(servoLLHug.getPosition() - .025, 0, 1)); //0
            servoLeftRightArm.setPosition(Range.clip(servoLeftRightArm.getPosition() - .025, 0, 1));
        }
        else if (gamepad2.dpad_right) {
            //servoLLHug.setPosition(Range.clip(servoLLHug.getPosition() + .025, 0, 1)); //.225

            servoLeftRightArm.setPosition(Range.clip(servoLeftRightArm.getPosition() + .025, 0, 1));
        }

        if (gamepad2.dpad_up) {
            servoUpDownArm.setPosition(Range.clip(servoUpDownArm.getPosition() + .025, 0, 1));
        }
        else if (gamepad2.dpad_down) {
            servoUpDownArm.setPosition(Range.clip(servoUpDownArm.getPosition() - .025, 0, 1));
        }*/

        if (gamepad2.dpad_up) {
            /*int encoderDiff = Math.abs(motorRelicTop.getCurrentPosition()) - Math.abs(motorRelicBottom.getCurrentPosition());
            double kP = .001;
            double PIDchange = Range.clip(kP * encoderDiff, -1, 1);
            if (PIDchange >= 0) {
                motorRelicTop.setPower(-1 + PIDchange);
                motorRelicBottom.setPower(-1);
            }
            else if (PIDchange < 0) {
                motorRelicTop.setPower(-1);
                motorRelicBottom.setPower(-1 - PIDchange);
            }*/
            motorRelicTop.setPower(-1);
            motorRelicBottom.setPower(-1);
        } else if (gamepad2.dpad_down) {
            /*int encoderDiff = Math.abs(motorRelicTop.getCurrentPosition()) - Math.abs(motorRelicBottom.getCurrentPosition());
            double kP = .001;
            double PIDchange = Range.clip(kP * encoderDiff, -1, 1);
            if (PIDchange >= 0) {
                motorRelicTop.setPower(1 - PIDchange);
                motorRelicBottom.setPower(1);
            }
            else if (PIDchange < 0) {
                motorRelicTop.setPower(1);
                motorRelicBottom.setPower(1 + PIDchange);
            }*/
            motorRelicTop.setPower(1);
            motorRelicBottom.setPower(1);
        } else {
            motorRelicBottom.setPower(0);
            motorRelicTop.setPower(0);
        }

        if (gamepad1.dpad_down) {
            servoRelicRot.setPosition(rotClosePos);
            //servoRelicRot.setPosition(Range.clip(servoRelicRot.getPosition() + .025, 0, 1));
        } else if (gamepad1.dpad_up) {
            servoRelicRot.setPosition(rotOpenPos);
            //servoRelicRot.setPosition(Range.clip(servoRelicRot.getPosition() - .025, 0, 1));
        } else if (gamepad1.dpad_left) {
            servoRelicRot.setPosition(.2);
        }

        if (gamepad1.left_bumper) {
            servoRelicGrab.setPosition(grabOpenPos);
        } else if (gamepad1.right_bumper) {
            servoRelicGrab.setPosition(grabClosePos);
        }

        if (gamepad2.x) { //all close
            //servoLRHug.setPosition(Range.clip(servoLRHug.getPosition() - .025, 0, 1)); //.775
            servoLLHug.setPosition(LLClose);
            servoLRHug.setPosition(LRClose);
            servoULHug.setPosition(ULClose);
            servoURHug.setPosition(URClose);
        }
        else if (gamepad2.b) { //all open
            //servoLRHug.setPosition(Range.clip(servoLRHug.getPosition() + .025, 0, 1)); //1
            servoLLHug.setPosition(LLOpen);
            servoLRHug.setPosition(LROpen);
            servoULHug.setPosition(ULOpen);
            servoURHug.setPosition(UROpen);
        }
        else if (gamepad2.y) { //all thread
            //servoLLHug.setPosition(Range.clip(servoLLHug.getPosition() - .025, 0, 1)); //.775
            servoLLHug.setPosition(LLThread);
            servoLRHug.setPosition(LRThread);
            servoULHug.setPosition(ULThread);
            servoURHug.setPosition(URThread);
        }
        else if (gamepad2.left_stick_button) {
            servoLLHug.setPosition(LLThread);
            servoLRHug.setPosition(LROpen);
            servoULHug.setPosition(ULThread);
            servoURHug.setPosition(UROpen);
        }
        else if (gamepad2.right_stick_button) {
            servoLLHug.setPosition(LLOpen);
            servoLRHug.setPosition(LRThread);
            servoULHug.setPosition(ULOpen);
            servoURHug.setPosition(URThread);
        }

        /*if (servoLLHug.getPosition() > .62) {
            servoLLHug.setPosition(LLOpen);
        }

        if (servoLRHug.getPosition() < .48) {
            servoLRHug.setPosition(LROpen);
        }*/

        if (gamepad2.dpad_right) { // bottom thread
            if (!isFlipped) {
                servoLLHug.setPosition(LLThread);
                servoLRHug.setPosition(LRThread);
            }
            else {
                servoURHug.setPosition(URThread);
                servoULHug.setPosition(ULThread);
            }
        }


        //left hug tighten
        if (gamepad2.left_bumper) {
            if (!isFlipped) {
                servoLRHug.setPosition(LRClose);
            }
            else {
                servoULHug.setPosition(ULClose);
            }
        }

        //left hug fully open
        else if (gamepad2.left_trigger > .1) {
            if (!isFlipped) {
                servoLRHug.setPosition(LROpen);
            }
            else {
                servoULHug.setPosition(ULOpen);
            }
        }

        //right hug tighten
        if (gamepad2.right_bumper) {
            if (!isFlipped) {
                servoLLHug.setPosition(LLClose);
            }
            else {
                servoURHug.setPosition(URClose);
            }
        }

        //right hug fully open
        else if (gamepad2.right_trigger > .1) {
            if (!isFlipped) {
                servoLLHug.setPosition(LLOpen);
            }
            else {
                servoURHug.setPosition(UROpen);
            }
        }

        if (gamepad2.a) {
            if (isFlipped) {
                servoFlip.setPosition(.025);}
            else {
                servoFlip.setPosition(.780);}

            isFlipped = !isFlipped;
            try {
                Thread.sleep(250);
            } catch (InterruptedException e) {
            }
        }



        //telemetry.addData("MotorFLEncoder", motorFL.getCurrentPosition());
        //telemetry.addData("MotorFREncoder", motorFR.getCurrentPosition());
        //telemetry.addData("MotorBLEncoder", motorBL.getCurrentPosition());
        //telemetry.addData("MotorBREncoder", motorBR.getCurrentPosition());
        //telemetry.addData("rangeL cm: ", getLeftDistance() + "");
        //telemetry.addData("rangeR cm: ", getRightDistance());
        //telemetry.addData("button enabled? ", buttonEnabled);
        //telemetry.addData("button", button.getVoltage());
        //telemetry.addData("isPressed?", isButtonPressed());
        telemetry.addData("motorScale: ", motorScale);
        //telemetry.addData("LiftL: ", motorLiftL.getCurrentPosition());
        //telemetry.addData("LiftR: ", motorLiftR.getCurrentPosition());
        telemetry.addData("servoLLHug Position: ", servoLLHug.getPosition());
        telemetry.addData("servoLRHug Position: ", servoLRHug.getPosition());
        telemetry.addData("servoULHug Position: ", servoULHug.getPosition());
        telemetry.addData("servoURHug Position: ", servoURHug.getPosition());
        telemetry.addData("servoLeftRight Position: ", servoLeftRightArm.getPosition());
        telemetry.addData("servoUpDown Position: ", servoUpDownArm.getPosition());
        telemetry.addData("motorRelicTop", motorRelicTop.getCurrentPosition());
        //telemetry.addData("motorRelicBottom", motorRelicBottom.getCurrentPosition());
        telemetry.addData("servoRelicRot", servoRelicRot.getPosition());
        telemetry.addData("servoRelicGrab", servoRelicGrab.getPosition());
        telemetry.addData("motorFR", motorFR.getCurrentPosition());
        telemetry.addData("motorFL", motorFL.getCurrentPosition());
        telemetry.addData("motorBR", motorBR.getCurrentPosition());
        telemetry.addData("motorBL", motorBL.getCurrentPosition());
    }


}
