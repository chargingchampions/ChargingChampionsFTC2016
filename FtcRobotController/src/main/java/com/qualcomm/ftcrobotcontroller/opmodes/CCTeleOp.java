package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

/**
 * Charging Champions TeleOp Mode
 * Enables control of the robot via the gamepad
 */
public class CCTeleOp extends OpMode {
    //4 Drive motors
    DcMotor motorFrontRight,motorFrontLeft, motorBackRight, motorBackLeft;

    // Motors for Hook and Pulley
    DcMotor motorHookRight, motorHookLeft, motorPulley;

    //Servos for linear Slider Tilt
    Servo linearSliderRight, linearSliderLeft;

    //Servos for ziplines
    Servo zipLineRight, zipLineLeft;

    /*
    //Servos for all-clear signal
    Servo allClearLeft, allClearRight;
   */

    //Servo for shelter
    Servo shelter;

    //Servo for ball sweep
    Servo ballSweep;


    /**
     * Constructor
     */
    public CCTeleOp() {

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
		 * We have 4 drive motors, 2 motors for the hook and 1 for pulley
		 * two servos "linear_right" and "linear_left" for linear slider tilt
		 * two servos "zipline_left" and "zipline_right" for operating the zipline
		 * one servo "shelter" for the Shelter mission
		 * one servo "ball_sweep" for clearing out debris
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

        //reverse direction of right hook will make the spools unwind in the same direction
        motorHookRight.setDirection(DcMotor.Direction.REVERSE);

        //reverse direction of pulley motor will result in channel going up
        motorPulley.setDirection(DcMotor.Direction.REVERSE);

        // Initialize Servo motors for the Linear Slider Tilt
        linearSliderRight = hardwareMap.servo.get("linear_right");
        linearSliderLeft = hardwareMap.servo.get("linear_left");
        zipLineRight = hardwareMap.servo.get("zipline_right");
        zipLineLeft = hardwareMap.servo.get("zipline_left");
        shelter = hardwareMap.servo.get("shelter");
        ballSweep = hardwareMap.servo.get("ball_sweep");
        //allClearLeft = hardwareMap.servo.get("all_left");
        //allClearRight = hardwareMap.servo.get("all_right");

    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {
        /*
		 * Gamepad 1 :
		 * Left and Right Stick: controls the drive motors
		 * Right Bumper: Right Zipline Servo  out
		 * Right Trigger : Right Zipline Servo in
		 * Left Bumper : Left Zipline Servo out
		 * Left Trigger : Left Zipline Servo in
		 * DPadUp : Drive forward up ramp
		 * DPadDown : Drive Backward down ramp
		 *
		 *
		 * Gamepad 2 :
		 * Left and Right Stick: controls the hook  & pulley motors
		 * Button A : Operate just pulley up
		 * Button X : Operate just pulley down
		 * Button Y : Operate just left and right hook motors up
		 * Button B : Operate just left and right hook motors down
		 * DPadUp : Linear Slider Forward
		 * DPadDown : Linear Slider Backward
		 * Right Bumper : Shelter forward
		 * Left Bumper : Shelter reverse
		 * Left Trigger : Sweep out
		 * Right Trigger : Sweep in
		 */

        //Read the gamepad1.left and right sticks for drive the robot
        float driveLeft = gamepad1.left_stick_y;
        float driveRight = gamepad1.right_stick_y;

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

        // write the values to the drive motors
        motorFrontRight.setPower(driveRight);
        motorBackRight.setPower(driveRight);
        motorFrontLeft.setPower(driveLeft);
        motorBackLeft.setPower(driveLeft);

        //motor speed for straight drive up the ramp
        if (gamepad1.dpad_down) {
            motorFrontRight.setPower(0.65);
            motorBackRight.setPower(0.65);
            motorBackLeft.setPower(0.65);
            motorFrontLeft.setPower(0.65);
        }
        else if (gamepad1.dpad_up) {
            motorFrontRight.setPower(-0.65);
            motorBackRight.setPower(-0.65);
            motorBackLeft.setPower(-0.65);
            motorFrontLeft.setPower(-0.65);
        }

        hook = (float)scaleInput(hook);

        // reduce power for the pulley motor to match the gear ratio
        // of the gears attached to the gear motors.
        float pulley = hook * 1 / 2;

        // if gamepad2.left and right stick were pressed, read the input
        // scale power and set power for the Hook and pulley motors
        if (hook != 0.0 && pulley != 0.0) {
            motorHookLeft.setPower(hook);
            motorHookRight.setPower(hook);
            motorPulley.setPower(pulley);
        }
        else {
            // operate the Pulley and Hook motors individually
            if (gamepad2.a) {
                motorPulley.setPower(0.85);
            } else if (gamepad2.x) {
                motorPulley.setPower(-0.85);
            } else if (gamepad2.y) {
                motorHookRight.setPower(0.85);
                motorHookLeft.setPower(0.85);
            } else if (gamepad2.b) {
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
        } else {
            linearSliderRight.setPosition(0.50);
            linearSliderLeft.setPosition(0.50);
        }

        //update zipLine on Right Side
        if (gamepad1.right_bumper) {
            zipLineRight.setPosition(0.6);
        } else if (gamepad1.right_trigger > 0.6) {
            zipLineRight.setPosition(0.4);
        } else {
            zipLineRight.setPosition(0.5);
        }

        //update zipLine on Left side
        if (gamepad1.left_bumper) {
            zipLineLeft.setPosition(0.4);
        } else if (gamepad1.left_trigger > 0.6) {
            zipLineLeft.setPosition(0.6);
        } else {
            zipLineLeft.setPosition(0.5);
        }
        //operate shelter servos
        if (gamepad2.right_bumper) {
            shelter.setPosition(0.7);
        } else if (gamepad2.left_bumper) {
            shelter.setPosition(0.3);
        } else {
            shelter.setPosition(0.5);
        }

        // operate ball sweep servo to clear debris
        if (gamepad2.left_trigger > 0.75) {
            ballSweep.setPosition(0.85);
        }
        else if (gamepad2.right_trigger > 0.75) {
            ballSweep.setPosition(0.15);
        }
        else {
            ballSweep.setPosition(0.5);
        }

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        //telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Zipline", "Right:  " + String.format("%.2f", zipLineRight.getPosition()));
        telemetry.addData("Zipline", "Left:  " + String.format("%.2f", zipLineLeft.getPosition()));
        telemetry.addData("Linear Slider", "Right:  " + String.format("%.2f", linearSliderRight.getPosition()));
        telemetry.addData("Linear Slider", "Left:  " + String.format("%.2f", linearSliderLeft.getPosition()));
        telemetry.addData("hook power", "hook:  " + String.format("%.2f", hook));
        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", driveLeft));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", driveRight));
        telemetry.addData("Ball Sweeper", "pwr: " + String.format("0.2f", ballSweep.getPosition()));
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
