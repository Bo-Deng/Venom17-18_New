package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

/**
 * Created by Hamza Ali on 2/24/2018.
 */
@TeleOp(name="PixyCamDataTest", group="opMode")
public class PixyCamDataTest extends CustomOpMode{

    public void init(){
        initStuff(hardwareMap);


        telemetry.addData("init ", "completed");
        telemetry.update();

        //setting Pixy's I2C Addresss
        pixyCam.setI2cAddress(I2cAddr.create7bit(0x50));

        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow (1, 26,
                I2cDeviceSynch.ReadMode.REPEAT);
        pixyCam.setReadWindow(readWindow);

        pixyCam.engage();
    }

    @Override
    public void loop(){
        pixyCam.engage();
        /*
        Use this to actually do something with the data we get
        ----------------------------------------------------------------
        0, 1     y              sync: 0xaa55=normal object, 0xaa56=color code object
        2, 3     y              checksum (sum of all 16-bit words 2-6, that is, bytes 4-13)
        4, 5     y              signature number
        6, 7     y              x center of object
        8, 9     y              y center of object
        10, 11   y              width of object
        12, 13   y              height of object
        */
        telemetry.addData("Byte 0", pixyCam.read8(0));
        telemetry.addData("Byte 1", pixyCam.read8(1));
        telemetry.addData("Byte 2", pixyCam.read8(2));
        telemetry.addData("Byte 3", pixyCam.read8(3));
        telemetry.addData("Byte 4", pixyCam.read8(4));
        telemetry.addData("Byte 5", pixyCam.read8(5));
        telemetry.addData("Byte 6", pixyCam.read8(6));
        telemetry.addData("Byte 7", pixyCam.read8(7));
        telemetry.addData("Byte 8", pixyCam.read8(8));
        telemetry.addData("Byte 9", pixyCam.read8(9));
        telemetry.addData("Byte 10", pixyCam.read8(10));
        telemetry.addData("Byte 11", pixyCam.read8(11));
        telemetry.addData("Byte 12", pixyCam.read8(12));
        telemetry.addData("Byte 13", pixyCam.read8(13));
        telemetry.update();
    }
}
