package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
/**
 * Created by rneervannan on 11/20/2015.
 */
public class MyAutonomous  extends OpMode {

    //Drive motors
    DcMotor motorFrontRight,motorFrontLeft, motorBackRight, motorBackLeft;

    /**
     * Constructor
     */
    public MyAutonomous() {

    }


    public void init() {
		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

		/*
		 *   We have 4 drive motors, 2 motors for the hook and 1 foor pulley
		 * We have two servos "linear_right" and "linear_left"
		 */
        motorFrontRight = hardwareMap.dcMotor.get("front_right");
        motorFrontLeft = hardwareMap.dcMotor.get("front_left");
        motorBackRight = hardwareMap.dcMotor.get("back_right");
        motorBackLeft = hardwareMap.dcMotor.get("back_left");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        motorFrontLeft = hardwareMap.dcMotor.get("front_left");
        motorFrontRight = hardwareMap.dcMotor.get("front_right");

        motorFrontLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFrontRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        //waitForStart();

        for(int i=0; i<4; i++) {
            motorFrontLeft.setPower(1.0);
            motorFrontRight.setPower(1.0);

            //sleep(1000);

           // leftMotor.setPower(0.5);
           // rightMotor.setPower(-0.5);

           // sleep(500);
        }

       // leftMotor.setPowerFloat();
       // rightMotor.setPowerFloat();

    }
}