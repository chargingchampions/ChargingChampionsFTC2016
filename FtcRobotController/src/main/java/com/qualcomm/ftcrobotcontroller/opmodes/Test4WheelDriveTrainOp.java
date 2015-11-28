package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class Test4WheelDriveTrainOp extends OpMode {
    //Drive motors
    DcMotor motorFrontRight,motorFrontLeft, motorBackRight, motorBackLeft;

    // Motors for Hook and Pulley
    DcMotor motorHookRight, motorHookLeft, motorPulley;

    //Servos for linear Slider Tilt
    Servo linearSliderRight, linearSliderLeft;

    //Servo rightZipLine;

    /**
     * Constructor
     */
    public Test4WheelDriveTrainOp() {

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
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
        motorHookLeft = hardwareMap.dcMotor.get("hook_left");
        motorHookRight = hardwareMap.dcMotor.get("hook_right");
        motorPulley = hardwareMap.dcMotor.get("pulley");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        //reverse direction of pulley motor will result in channel going up
        motorPulley.setDirection(DcMotor.Direction.REVERSE);

        // Initialize Servo motors for the Linear Slider Tilt
        linearSliderRight = hardwareMap.servo.get("linear_right");
        linearSliderLeft = hardwareMap.servo.get("linear_left");
        //rightZipLine = hardwareMap.servo.get("zipline");


    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {
        /*
		 * Gamepad 1
		 *
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */

        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.
        // throttle:  left_stick_y ranges from -1 to 1, where -1 is full up,  and 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left and 1 is full right
        float driveLeft = -gamepad1.left_stick_y;
        float driveRight = -gamepad1.right_stick_y;

        //Taking the average of left and right stick on gamepad2 for the hook movement
        float hook = ((-gamepad2.right_stick_y)/2)+((-gamepad2.left_stick_y)/2);


        // clip the right/left values so that the values never exceed +/- 1
        driveRight = Range.clip(driveRight, -1, 1);
        driveLeft = Range.clip(driveLeft, -1, 1);
        hook = Range.clip(hook, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        driveRight = (float)scaleInput(driveRight*0.75);
        driveLeft = (float)scaleInput(driveLeft*0.75);
        hook = (float)scaleInput(hook);

        // reduce power for the pulley motor to match the gear ratio
        // of the gears attached to the gear motors.
        float pulley = hook * 1 / 2;

        // write the values to the drive motors
        motorFrontRight.setPower(driveRight);
        motorBackRight.setPower(driveRight);
        motorFrontLeft.setPower(driveLeft);
        motorBackLeft.setPower(driveLeft);

        if (gamepad1.dpad_up) {
            motorFrontRight.setPower(0.65);
            motorBackRight.setPower(0.65);
            motorBackLeft.setPower(0.65);
            motorFrontLeft.setPower(0.65);
        }
        else if (gamepad1.dpad_down) {
            motorFrontRight.setPower(-0.65);
            motorBackRight.setPower(-0.65);
            motorBackLeft.setPower(-0.65);
            motorFrontLeft.setPower(-0.65);
        }
        // set power for the Hook and pulley motors
        if (hook != 0.0 && pulley != 0.0) {
            motorHookLeft.setPower(hook);
            motorHookRight.setPower(hook);
            motorPulley.setPower(pulley);
        }
        else {
            if (gamepad2.a) {
                motorPulley.setPower(0.85);
            }

            else if (gamepad2.y) {
                motorHookRight.setPower(-0.85);
                motorHookLeft.setPower(-0.85);
            } else {
                motorHookLeft.setPower(0.0);
                motorHookRight.setPower(0.0);
                motorPulley.setPower(0.0);
            }
        }

        // update the position of the linear tilt.
        if (gamepad2.dpad_up ) {
            linearSliderRight.setPosition(0.95);
            linearSliderLeft.setPosition(0.05);
        } else if (gamepad2.dpad_down) {
            linearSliderRight.setPosition(0.05);
            linearSliderLeft.setPosition(0.95);
        } else if (gamepad2.left_bumper){
            linearSliderRight.setPosition(0.50);
            linearSliderLeft.setPosition(0.50);
        }
/*
        if (gamepad1.right_bumper) {
            rightZipLine.setPosition(0.6);
        } else {
            rightZipLine.setPosition(0.5);
        }*/

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        //telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Servo", "Right:  " + String.format("%.2f", linearSliderRight.getPosition()));
        telemetry.addData("Servo", "Left:  " + String.format("%.2f", linearSliderLeft.getPosition()));
        telemetry.addData("hook power", "hook:  " + String.format("%.2f", hook));
        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", driveLeft));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", driveRight));
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {


    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}
