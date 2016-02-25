package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Charging Champions Autonomous Mode
 * Program is run when the robot is parked in the red zone at start
 */
public class ODSTest extends LinearOpMode
{
    // Set initial encoder positions for left and right motors
    int leftEncoderTarget = 0, rightEncoderTarget = 0;

    int rightEncoderTicks = 0, leftEncoderTicks = 0;
    private int step;
    private boolean robotMoving;
    private boolean bShelter = false;
    //Servo for shelter
    Servo shelter;

    // OpticalDistanceSensor to locate the white line
    // near the beacon repair zone.
    OpticalDistanceSensor opticalDistanceSensor;

    // Initialize left and right drive motors
    DcMotor motorBackRight, motorFrontRight;
    DcMotor motorBackLeft, motorFrontLeft;

    /*
     * Get the currentPosition of the motor to determine if
     * it has reached the target position needed
     */
    public boolean moveComplete() throws InterruptedException
    {
        waitOneFullHardwareCycle();
        return 	(Math.abs(motorFrontLeft.getCurrentPosition() - leftEncoderTarget) < 5) ||
                (Math.abs(motorFrontRight.getCurrentPosition() - rightEncoderTarget) < 5) ||
                (Math.abs(motorBackLeft.getCurrentPosition() - leftEncoderTarget) < 5) ||
                (Math.abs(motorBackRight.getCurrentPosition() - rightEncoderTarget) < 5);
    }

    /*
     * Set motor power to 0 at the end of Autonomous
     */
    public void autoShutdown() throws InterruptedException
    {

        //shelter.setPosition(0.3);
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


    public void robotInit() throws InterruptedException
    {
        // Initialize motors from hardware mapping.
        motorBackLeft = hardwareMap.dcMotor.get("back_left");
        motorBackRight = hardwareMap.dcMotor.get("back_right");
        motorFrontLeft = hardwareMap.dcMotor.get("front_left");
        motorFrontRight = hardwareMap.dcMotor.get("front_right");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");
        shelter = hardwareMap.servo.get("shelter");

        leftEncoderTarget = 0;
        rightEncoderTarget = 0;
        int encoderResetThreshold = 3;

        motorBackLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFrontRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        motorBackLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBackRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFrontLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFrontRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);


        telemetry.addData("FL getCurrentPosition: ", motorFrontLeft.getCurrentPosition());
        telemetry.addData("FR getCurrentPosition: ", motorFrontRight.getCurrentPosition());
        telemetry.addData("BL getCurrentPosition: ", motorBackLeft.getCurrentPosition());
        telemetry.addData("BR getCurrentPosition: ", motorBackRight.getCurrentPosition());

        while	((Math.abs(motorBackLeft.getCurrentPosition()) > encoderResetThreshold) &&
                (Math.abs(motorBackRight.getCurrentPosition()) > encoderResetThreshold)){
            telemetry.addData("FL getCurrentPosition: ", motorFrontLeft.getCurrentPosition());
            telemetry.addData("FR getCurrentPosition: ", motorFrontRight.getCurrentPosition());
            telemetry.addData("BL getCurrentPosition: ", motorBackLeft.getCurrentPosition());
            telemetry.addData("BR getCurrentPosition: ", motorBackRight.getCurrentPosition());

            waitOneFullHardwareCycle();
        }
        step = 1;
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        this.robotInit();
        // wait for the start button to be pressed
        waitForStart();

        //waitOneFullHardwareCycle();

        double reflectance = opticalDistanceSensor.getLightDetected();
        telemetry.addData("2 ", "ODS:  " + String.format("%.2f",  opticalDistanceSensor.getLightDetected()));

        while (opModeIsActive()) {
            telemetry.addData("3 ", "ODS:  " + String.format("%.2f",  opticalDistanceSensor.getLightDetected()));
           /* if (a_ods_white_tape_detected()) {
                motorBackLeft.setPower(0.5);
                motorFrontLeft.setPower(0.5);
                motorBackRight.setPower(0.5);
                motorFrontRight.setPower(0.5);
                telemetry.addData("3", "Here:  " + String.format("%.2f",  opticalDistanceSensor.getLightDetected()));
            } else {
                motorBackRight.setPower(0.0);
                motorBackLeft.setPower(0.0);
                motorFrontLeft.setPower(0.0);
                motorFrontRight.setPower(0.0);
                        telemetry.addData("5", "Here:  " + String.format("%.2f",  opticalDistanceSensor.getLightDetected()));
            }
            motorBackRight.setPower(0.0);
            motorBackLeft.setPower(0.0);
            motorFrontLeft.setPower(0.0);
            motorFrontLeft.setPower(0.0);
            telemetry.addData("4 ", "here1:  " + String.format("%.2f",  opticalDistanceSensor.getLightDetected()));*/

            waitOneFullHardwareCycle();
        }
    }


    /**
     * Access whether the ODS is detecting white tape.
     */
    boolean a_ods_white_tape_detected () {
        boolean l_return = false;

        if (opticalDistanceSensor != null) {
            double value = opticalDistanceSensor.getLightDetected();
            int value1 = opticalDistanceSensor.getLightDetectedRaw();
            telemetry.addData("a_ods_white_tape_detected "," " + String.format("%.2f ", value));
            telemetry.addData("raw light "," " + String.format("%d ", value1));
            // Is the amount of light detected above the threshold for white tape?
            if (value > 0.8) {
                l_return = true;
                telemetry.addData("returning ","true " + String.format("%.2f", value));
            }
        }
        return l_return;
    }

}