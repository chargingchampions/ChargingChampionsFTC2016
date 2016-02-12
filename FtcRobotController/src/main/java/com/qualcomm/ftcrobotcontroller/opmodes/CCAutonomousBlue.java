package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Charging Champions Autonomous Mode
 * Program is run when the robot is parked in the blue zone at start
 */
public class CCAutonomousBlue extends LinearOpMode
{
	// Set initial encoder positions for left and right motors
	int leftEncoderTarget = 0, rightEncoderTarget = 0;

	int rightEncoderTicks = 0, leftEncoderTicks = 0;
	int step;
	boolean robotMoving;

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

		//while ((Math.abs(motorBackLeft.getCurrentPosition()) > encoderResetThreshold) &&
		while	((Math.abs(motorBackLeft.getCurrentPosition()) > encoderResetThreshold) &&
				(Math.abs(motorBackRight.getCurrentPosition()) > encoderResetThreshold)){
			telemetry.addData("getCurrentPosition: ", motorFrontLeft.getCurrentPosition());
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
		while (opModeIsActive()) {
			switch (step) {
				case 1:
					//Drive the robot forward from the start zone
					if (!robotMoving) {
						// Robot moves 50 inches
                       /* int distance = caculateEncoderTicks(50);
                        robotMove(distance, distance);
                        telemetry.addData("A.  ", "distance:  " + distance);*/
						robotMove(4500, 4500);
						robotMoving = true;
					}

					reflectance = opticalDistanceSensor.getLightDetected();
					telemetry.addData("1 ", "ODS:  " + String.format("%.2f", reflectance));
					telemetry.addData("MoveTo Back ", "Left: " + motorBackLeft.getTargetPosition() + " Right: " + motorBackRight.getTargetPosition());
					telemetry.addData("MoveTo Front ", "Left: " + motorFrontLeft.getTargetPosition() + " Right: " + motorFrontRight.getTargetPosition());
					if (!moveComplete()) {

						telemetry.addData("PositionBack:", "Left: " + motorBackLeft.getCurrentPosition() + " Right: " + motorBackRight.getCurrentPosition());
						telemetry.addData("PositionFront:", "Left: " + motorFrontLeft.getCurrentPosition() + " Right: " + motorFrontRight.getCurrentPosition());
					} else {
						telemetry.addData("SucceededPositionBack", "Left: " + motorBackLeft.getCurrentPosition() + " Right: " + motorBackRight.getCurrentPosition());
						telemetry.addData("SucceededPositionFront", "Left: " + motorFrontLeft.getCurrentPosition() + " Right: " + motorFrontRight.getCurrentPosition());
						robotMoving = false;
						step++;
					}
					break;

				case 2:
					// Make a 90 degree right return
					telemetry.addData("Drive:", motorBackLeft.toString());
					telemetry.addData("Status", "Turning right");

					if (!robotMoving) {
						robotMove(-2000,1000);
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
						//robotMove(caculateEncoderTicks(36), caculateEncoderTicks(36));
						robotMove(6400,6400);
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
					// Drive to the beacon repair zone
					telemetry.addData("Status", "make a slight right");

					if (!robotMoving) {
						//robotMove(caculateEncoderTicks(36), caculateEncoderTicks(36));
						robotMove(-2000,1000);
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
					telemetry.addData("4 ", "ODS:  " + String.format("%.2f", reflectance));
					break;
				case 5:
					//Rotate the robot to position the shelter arm ready to drop in to the basket
					telemetry.addData("Status", "Drive forward ");

					if (!robotMoving) {
						robotMove(3000,3000);
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
					telemetry.addData("5 ", "ODS:  " + String.format("%.2f", reflectance));
					break;

				default:
					autoShutdown();
					break;
			}
			waitOneFullHardwareCycle();
		}
	}

	/*
     * Encoderticks = (360/Circumference)* DistanceToTravel
     * Diameter of the wheel is 4"
     */
	public int caculateEncoderTicks(int distanceToTravel)
	{
		return (int) ((360/(Math.PI*4))* distanceToTravel);
	}
	/*
	 * Move each motor the specified distance (value) in encoder ticks
	 */
	public void robotMove(int leftEncoder, int rightEncoder)
	{
		leftEncoderTarget += leftEncoder;
		rightEncoderTarget += rightEncoder;
		motorBackLeft.setTargetPosition(leftEncoderTarget);
		motorBackRight.setTargetPosition(rightEncoderTarget);
		motorFrontLeft.setTargetPosition(leftEncoderTarget);
		motorFrontRight.setTargetPosition(rightEncoderTarget);
		telemetry.addData("B ", "leftEncoderTarget:  " +leftEncoderTarget);

		motorBackLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
		motorBackRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
		motorFrontLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
		motorFrontRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

		motorBackLeft.setPower(1.0);
		motorBackRight.setPower(1.0);
		motorFrontLeft.setPower(1.0);
		motorFrontRight.setPower(1.0);
	}

}