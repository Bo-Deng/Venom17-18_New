package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Bo on 9/18/2017.
 */
@TeleOp(name="SensorTesting", group="opMode")
public class SensorTesting extends OpMode {
    IMU imu;
    ModernRoboticsI2cRangeSensor rangeL;
    ModernRoboticsI2cRangeSensor rangeR;


    public void init() {
        imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
        imu.IMUinit(hardwareMap);
        rangeL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeL");
        rangeR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeR");

        telemetry.addData("Yaw: ", imu.getYaw());
        telemetry.addData("RangeL: ", rangeL.getDistance(DistanceUnit.INCH));
        telemetry.addData("RangeR: ", rangeR.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }

    public void loop() {

        telemetry.addData("Yaw: ", imu.getYaw());
        telemetry.addData("RangeL: ", rangeL.getDistance(DistanceUnit.INCH));
        telemetry.addData("RangeR: ", rangeR.getDistance(DistanceUnit.INCH));
    }
}
