package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/*
 * An example linear op mode where the pushbot
 * will drive in a square pattern using sleep()
 * and a for loop.
 */
public class PushBotSquare extends LinearOpMode {
    //Drive motors
    DcMotor motorFrontRight,motorFrontLeft, motorBackRight, motorBackLeft;
    double speed = 1.0;
    double distance = 5;
    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("front_right");
        motorFrontLeft = hardwareMap.dcMotor.get("front_left");
        motorBackRight = hardwareMap.dcMotor.get("back_right");
        motorBackLeft = hardwareMap.dcMotor.get("back_left");

        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFrontLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBackRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBackLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        waitForStart();

        // Reset enoders to zero
        motorFrontLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFrontRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        waitOneFullHardwareCycle(); // We use these in attempt to gain stability.
        // Wait for encoders reset
        while(motorFrontLeft.getCurrentPosition() != 0 && motorFrontRight.getCurrentPosition() != 0  &&
                motorBackLeft.getCurrentPosition() != 0 && motorBackRight.getCurrentPosition() != 0){}

        speed = speed / 100; // For new MR controller
        // Handle negative speed input to routine.
        if(speed < 0 && distance > 0){
            distance = Math.abs(distance) * -1;
        }


        distance = distance / (4.88 * Math.PI);
        double targetDistance = distance * 1220;

        motorFrontLeft.setTargetPosition((int) targetDistance);
        motorFrontRight.setTargetPosition((int) targetDistance);
        motorBackLeft.setTargetPosition((int) targetDistance);
        motorBackRight.setTargetPosition((int) targetDistance);
        motorFrontRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        waitOneFullHardwareCycle();
        motorFrontLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackLeft.setPower(speed);
        motorBackRight.setPower(speed);



        waitOneFullHardwareCycle();

        /*while(motorFrontLeft.isBusy() || motorFrontRight.isBusy() ||
                motorBackLeft.isBusy() || motorBackRight.isBusy()) { */
            telemetry.addData("leftMotor", motorFrontLeft.getCurrentPosition());
            telemetry.addData("rightMotor", motorFrontRight.getCurrentPosition());
            telemetry.addData("leftMotor", motorBackLeft.getCurrentPosition());
            telemetry.addData("rightMotor", motorBackRight.getCurrentPosition());
            telemetry.addData("speed", speed);
       // }
        waitOneFullHardwareCycle();
        //motorFrontLeft.setPower(0.0);
        //motorFrontRight.setPower(0.0);
        //motorBackLeft.setPower(0.0);
        //motorBackRight.setPower(0.0);
/*
        for(int i=0; i<4; i++) {
            motorFrontLeft.setPower(0.9);
            motorFrontRight.setPower(0.9);
            motorBackLeft.setPower(0.9);
            motorBackRight.setPower(0.9);

            sleep(100);

            motorFrontLeft.setPower(0.5);
            motorFrontRight.setPower(-0.5);
            motorBackLeft.setPower(0.5);
            motorBackRight.setPower(-0.5);

            sleep(500);
        }

        motorFrontLeft.setPowerFloat();
        motorFrontRight.setPowerFloat();
        motorBackLeft.setPowerFloat();
        motorBackRight.setPowerFloat();

        motorFrontRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFrontRight.setTargetPosition(1220);
        motorFrontLeft.setTargetPosition(1220);
        motorFrontRight.setPower(1);
        motorFrontLeft.setPower(1);

        motorBackRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorBackRight.setTargetPosition(1220);
        motorBackLeft.setTargetPosition(1220);
        motorBackRight.setPower(1);
        motorBackLeft.setPower(1);

        waitOneFullHardwareCycle();
        leftMotor.setPower(speed);
        rightMotor.setPower(speed);

        waitOneFullHardwareCycle();

        while(leftMotor.isBusy() || rightMotor.isBusy() || closeToTargetLeft(targetDistance)){
            telemetry.addData("leftMotor", leftMotor.getCurrentPosition());
            telemetry.addData("rightMotor", rightMotor.getCurrentPosition());
            telemetry.addData("speed", speed);
        }
        waitOneFullHardwareCycle();
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);


        telemetry.addData("left tgt pwr", "back left  pwr: " + String.format("%.2f", motorBackLeft.getPower()));
        telemetry.addData("right tgt pwr", "back right pwr: " + String.format("%.2f", motorBackRight.getPower()));
        telemetry.addData("left tgt pwr", "front left  pwr: " + String.format("%.2f", motorFrontLeft.getPower()));
        telemetry.addData("right tgt pwr", "front right pwr: " + String.format("%.2f", motorFrontRight.getPower())); */
    }


    public void drive(double distance, double speed) throws InterruptedException
    {

    }
}
