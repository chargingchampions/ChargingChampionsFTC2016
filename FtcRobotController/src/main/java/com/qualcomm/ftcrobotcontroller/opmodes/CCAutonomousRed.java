package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Charging Champions Autonomous Mode
 * Program is run when the robot is parked in the red zone at start
 */
public class CCAutonomousRed extends LinearOpMode
{
	// Set initial encoder positions for left and right motors
	int leftEncoderTarget = 0;
	int rightEncoderTarget = 0;

	private int step;
	private boolean robotMoving;

	// OpticalDistanceSensor to locate the white line
	// near the beacon repair zone.
	OpticalDistanceSensor opticalDistanceSensor;

	// Initialize left and right drive motors
	DcMotor motorRight, motorFrontRight;
	DcMotor motorLeft, motorFrontLeft;

	/*
	 * Get the currentPosition of the motor to determine if
	 * it has reached the target position needed
	 */
	public boolean moveComplete() throws InterruptedException
	{
		waitOneFullHardwareCycle();
		return 	(Math.abs(motorFrontLeft.getCurrentPosition() - leftEncoderTarget) < 5) &&
				(Math.abs(motorFrontRight.getCurrentPosition() - rightEncoderTarget) < 5);
	}

	/*
	 * Set motor power to 0 at the end of Autonomous
	 */
	public void autoShutdown()
	{
		motorRight.setPower(0);
		motorLeft.setPower(0);
		motorFrontRight.setPower(0);
		motorFrontLeft.setPower(0);
		telemetry.addData("Autonomous", "Completed!");
	}


	public void robotInit() throws InterruptedException
	{
		// Initialize motors from hardware mapping.
		motorLeft = hardwareMap.dcMotor.get("back_left");
		motorRight = hardwareMap.dcMotor.get("back_right");
		motorFrontLeft = hardwareMap.dcMotor.get("front_left");
		motorFrontRight = hardwareMap.dcMotor.get("front_right");
		motorRight.setDirection(DcMotor.Direction.REVERSE);
		motorFrontRight.setDirection(DcMotor.Direction.REVERSE);

		opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");

		leftEncoderTarget = 0;
		rightEncoderTarget = 0;

		int encoderResetThreshold = 3;
		motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		motorFrontLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);

		while	(Math.abs(motorFrontLeft.getCurrentPosition()) > encoderResetThreshold) {
			waitForNextHardwareCycle();
		}

		motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		motorFrontRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);

		while ((Math.abs(motorFrontRight.getCurrentPosition()) > encoderResetThreshold)) {
			waitForNextHardwareCycle();
		}
		step = 1;
	}


	@Override
	public void runOpMode() throws InterruptedException
	{
		this.robotInit();
		// wait for the start button to be pressed
		waitForStart();
		motorLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
		motorRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
		motorFrontLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
		motorFrontRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
		waitOneFullHardwareCycle();

		double reflectance = opticalDistanceSensor.getLightDetected();
		while (opModeIsActive()) {
			switch (step) {
				case 1:
					//Drive the robot forward from the start zone
					if (!robotMoving) {
						robotMove(9800, 9800);
						robotMoving = true;
					}

					reflectance = opticalDistanceSensor.getLightDetected();
					telemetry.addData("1 ", "ODS:  " + String.format("%.2f", reflectance));
					telemetry.addData("MoveTo", "Left: " + motorLeft.getTargetPosition() + " Right: " + motorRight.getTargetPosition());
					telemetry.addData("MoveTo Front", "Left: " + motorFrontLeft.getTargetPosition() + " Right: " + motorFrontRight.getTargetPosition());
					if (!moveComplete()) {

						telemetry.addData("Position:", "Left: " + motorLeft.getCurrentPosition() + " Right: " + motorRight.getCurrentPosition());
						telemetry.addData("PositionFront:", "Left: " + motorFrontLeft.getCurrentPosition() + " Right: " + motorFrontRight.getCurrentPosition());
					} else {
						telemetry.addData("SucceededPosition", "Left: " + motorLeft.getCurrentPosition() + " Right: " + motorRight.getCurrentPosition());
						telemetry.addData("SucceededPositionFront", "Left: " + motorFrontLeft.getCurrentPosition() + " Right: " + motorFrontRight.getCurrentPosition());
						robotMoving = false;
						step++;
					}
					break;

				case 2:
					// Make a 90 degree left return
					telemetry.addData("Drive:", motorLeft.toString());
					telemetry.addData("Status", "Turning Left");

					if (!robotMoving) {
						robotMove(-2000,2000);
						robotMoving = true;
					}

					if (!moveComplete()) {
						telemetry.addData("SucceededPosition", "Left: " + motorFrontLeft.getCurrentPosition() + " Right: " + motorFrontRight.getCurrentPosition());
					} else {
						telemetry.addData("SucceededPosition", "Left: " + motorFrontLeft.getCurrentPosition() + " Right: " + motorFrontRight.getCurrentPosition());
						robotMoving = false;
						step++;
					}
					reflectance = opticalDistanceSensor.getLightDetected();
					telemetry.addData("2 ", "ODS:  " + String.format("%.2f", reflectance));

					break;

				case 3:
					// Drive to the beacon repair zone
					telemetry.addData("Status", "Driving forward ");

					if (!robotMoving) {
						robotMove(7000,7000);
						robotMoving = true;
					}

					if (!moveComplete()) {
						telemetry.addData("Position:", "Left: " + motorFrontLeft.getCurrentPosition() + " Right: " + motorFrontRight.getCurrentPosition());
					} else {
						reflectance = opticalDistanceSensor.getLightDetected();
						telemetry.addData("SucceededPosition", "Left: " + motorFrontLeft.getCurrentPosition() + " Right: " + motorFrontRight.getCurrentPosition());
						telemetry.addData("final ", "ODS:  " + String.format("%.2f", reflectance));
						robotMoving = false;
						step++;
					}

					reflectance = opticalDistanceSensor.getLightDetected();
					telemetry.addData("3 ", "ODS:  " + String.format("%.2f", reflectance));
					break;
				case 4:
					//Rotate the robot to position the shelter arm ready to drop in to the basket
					telemetry.addData("Status", "Rotating in place ");

					if (!robotMoving) {
						robotMove(0,7000);
						robotMoving = true;
					}

					if (!moveComplete()) {
						telemetry.addData("Position:", "Left: " + motorFrontLeft.getCurrentPosition() + " Right: " + motorFrontRight.getCurrentPosition());
					} else {
						reflectance = opticalDistanceSensor.getLightDetected();
						telemetry.addData("SucceededPosition", "Left: " + motorFrontLeft.getCurrentPosition() + " Right: " + motorFrontRight.getCurrentPosition());
						telemetry.addData("final ", "ODS:  " + String.format("%.2f", reflectance));
						robotMoving = false;
						step++;
						autoShutdown();
					}
					reflectance = opticalDistanceSensor.getLightDetected();
					telemetry.addData("3 ", "ODS:  " + String.format("%.2f", reflectance));
					break;

				default:
					break;
			}
			waitOneFullHardwareCycle();
		}
	}

	/*
	 * Move each motor the specified distance (value) in encoder ticks
	 */
	public void robotMove(int leftEncoder, int rightEncoder)
	{
		leftEncoderTarget += leftEncoder;
		rightEncoderTarget += rightEncoder;
		motorLeft.setTargetPosition(leftEncoderTarget);
		motorRight.setTargetPosition(rightEncoderTarget);
		motorFrontLeft.setTargetPosition(leftEncoderTarget);
		motorFrontRight.setTargetPosition(rightEncoderTarget);
		motorLeft.setPower(1.0);
		motorRight.setPower(1.0);
		motorFrontLeft.setPower(1.0);
		motorFrontRight.setPower(1.0);
	}
}