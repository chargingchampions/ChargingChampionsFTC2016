package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
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
        /*motorFrontRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFrontLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBackRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBackLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS); */
        waitForStart();
        // Reset enoders to zero
      /*  motorFrontLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFrontRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        waitOneFullHardwareCycle(); // We use these in attempt to gain stability.


        motorFrontLeft.setTargetPosition(2440);
        motorFrontRight.setTargetPosition(2440);
        motorBackLeft.setTargetPosition(2440);
        motorBackRight.setTargetPosition(2440);

        motorFrontRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setPower(0.5);
        motorFrontRight.setPower(0.5);
        motorBackLeft.setPower(0.5);
        motorBackRight.setPower(0.5);*/

        double reflectance = opticalDistanceSensor.getLightDetected();

        while (reflectance < 0.189) {
            motorFrontLeft.setPower(-0.2);
            motorFrontRight.setPower(-0.2);
            motorBackLeft.setPower(-0.2);
            motorBackRight.setPower(-0.2);
        }
        if (reflectance >= 0.19) {
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

        telemetry.addData("1 ", "ODS:  " + String.format("%.2f", reflectance));
        telemetry.addData("2 ", "motorFrontLeft:  " + String.format("%d", motorFrontLeft.getCurrentPosition()));
        telemetry.addData("3 ", "motorFrontRight:  " + String.format("%d", motorFrontRight.getCurrentPosition()));
        telemetry.addData("4 ", "motorBackLeft:  " + String.format("%d", motorBackLeft.getCurrentPosition()));
        telemetry.addData("5 ", "motorBackRight:  " + String.format("%d", motorBackRight.getCurrentPosition()));
    }


}
