package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by rneervannan1 on 11/17/2015.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class TouchLinearOpMode extends LinearOpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    TouchSensor touchsensor;

    @Override
    public void runOpMode() throws InterruptedException {

        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        touchsensor = hardwareMap.touchSensor.get("touch_sensor");

        waitForStart();

        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);

        // idle until the touch sensor is pressed
        while (!touchsensor.isPressed()) {
            telemetry.addData("isPressed",String.valueOf(touchsensor.isPressed()));
            waitForNextHardwareCycle();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        telemetry.addData("isPressed",String.valueOf(touchsensor.isPressed()));
    }
}