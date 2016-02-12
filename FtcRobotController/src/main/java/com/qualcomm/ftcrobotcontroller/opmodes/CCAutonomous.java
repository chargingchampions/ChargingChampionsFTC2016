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
    // Set initial encoder positions for left and right motors
    int leftEncoderTarget = 0, rightEncoderTarget = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");

        motorFrontRight = hardwareMap.dcMotor.get("front_right");
        motorFrontLeft = hardwareMap.dcMotor.get("front_left");
        motorBackRight = hardwareMap.dcMotor.get("back_right");
        motorBackLeft = hardwareMap.dcMotor.get("back_left");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFrontLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBackRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBackLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFrontLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFrontRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        waitOneFullHardwareCycle(); // We use these in attempt to gain stability.
        waitForStart();
        // Reset enoders to zero

        motorFrontLeft.setTargetPosition(2440);
        motorFrontRight.setTargetPosition(2440);
        motorBackLeft.setTargetPosition(2440);
        motorBackRight.setTargetPosition(2440);

        motorFrontRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setPower(1.0);
        motorFrontRight.setPower(1.0);
        motorBackLeft.setPower(1.0);
        motorBackRight.setPower(1.0);

        /*
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
        } */

       // telemetry.addData("1 ", "ODS:  " + String.format("%.2f", reflectance));
        telemetry.addData("2 ", "motorFrontLeft:  " + String.format("%d", motorFrontLeft.getCurrentPosition()));
        telemetry.addData("3 ", "motorFrontRight:  " + String.format("%d", motorFrontRight.getCurrentPosition()));
        telemetry.addData("4 ", "motorBackLeft:  " + String.format("%d", motorBackLeft.getCurrentPosition()));
        telemetry.addData("5 ", "motorBackRight:  " + String.format("%d", motorBackRight.getCurrentPosition()));
    }

    public boolean moveComplete() throws InterruptedException
    {
        waitOneFullHardwareCycle();
        return 	(Math.abs(motorFrontLeft.getCurrentPosition() - leftEncoderTarget) < 5) &&
                (Math.abs(motorFrontRight.getCurrentPosition() - rightEncoderTarget) < 5);
    }

    public void autoShutdown() throws InterruptedException
    {
        //Reset the motor encoders and then stop the motor
        motorBackLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFrontRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        waitOneFullHardwareCycle();

        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);

        motorBackLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorFrontLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorBackRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorFrontRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        waitOneFullHardwareCycle();

        telemetry.addData("motor mode:", motorBackLeft.getMode().toString());
        telemetry.addData("Autonomous", "Completed!");
    }

}
