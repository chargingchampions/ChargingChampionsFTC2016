package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by rneervannan on 12/7/2015.
 */
public class CCAutonomous extends LinearOpMode {
    OpticalDistanceSensor opticalDistanceSensor;
    //Drive motors
    DcMotor motorFrontRight,motorFrontLeft, motorBackRight, motorBackLeft;

    @Override
    public void runOpMode() throws InterruptedException {

        opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");

        motorFrontRight = hardwareMap.dcMotor.get("front_right");
        motorFrontLeft = hardwareMap.dcMotor.get("front_left");
        motorBackRight = hardwareMap.dcMotor.get("back_right");
        motorBackLeft = hardwareMap.dcMotor.get("back_left");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        double reflectance = opticalDistanceSensor.getLightDetected();
        waitForStart();
        if (reflectance < 0.189) {
            motorFrontLeft.setPower(-0.2);
            motorFrontRight.setPower(-0.2);
            motorBackLeft.setPower(-0.2);
            motorBackRight.setPower(-0.2);
        } else if (reflectance >= 0.19){
            motorFrontRight.setPower(-0.3);
            motorBackRight.setPower(-0.3);
            sleep(2000);
            motorFrontLeft.setPower(-0.3);
            motorBackLeft.setPower(-0.3);
            sleep(500);
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
        }
    }


}
