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
public class CCAutoRed2 extends LinearOpMode
{
    // Set initial encoder positions for left and right motors
    int leftEncoderTarget = 0, rightEncoderTarget = 0;
    double currentODSReading;

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

        double initialODSReading = opticalDistanceSensor.getLightDetected();
        while (opModeIsActive()) {
            switch (step) {
                case 1:
                    //Drive the robot forward from the start zone
                    if (!robotMoving) {
                        // Robot moves 50 inches
                       /* int distance = caculateEncoderTicks(50);
                        robotMove(distance, distance);
                        telemetry.addData("A.  ", "distance:  " + distance);*/
                        robotMove(10800, 10800);
                        robotMoving = true;
                    }

                    initialODSReading = opticalDistanceSensor.getLightDetected();
                    telemetry.addData("1 ", "ODS:  " + String.format("%.2f", initialODSReading));
                    telemetry.addData("MoveTo Back ", "Left: " + motorBackLeft.getTargetPosition() + " Right: " + motorBackRight.getTargetPosition());
                    telemetry.addData("MoveTo Front ", "Left: " + motorFrontLeft.getTargetPosition() + " Right: " + motorFrontRight.getTargetPosition());
                    if (!moveComplete()) {
                        telemetry.addData("not ", "ODS:  " + String.format("%.2f", opticalDistanceSensor.getLightDetected()));
                        telemetry.addData("PositionBack:", "Left: " + motorBackLeft.getCurrentPosition() + " Right: " + motorBackRight.getCurrentPosition());
                        telemetry.addData("PositionFront:", "Left: " + motorFrontLeft.getCurrentPosition() + " Right: " + motorFrontRight.getCurrentPosition());
                    } else {
                        telemetry.addData("complete ", "ODS:  " + String.format("%.2f", opticalDistanceSensor.getLightDetected()));
                        telemetry.addData("SucceededPositionBack", "Left: " + motorBackLeft.getCurrentPosition() + " Right: " + motorBackRight.getCurrentPosition());
                        telemetry.addData("SucceededPositionFront", "Left: " + motorFrontLeft.getCurrentPosition() + " Right: " + motorFrontRight.getCurrentPosition());
                        robotMoving = false;
                        step++;
                    }
                    break;


                case 2:
                    //Rotate the robot to position the shelter arm ready to drop in to the basket
                    telemetry.addData("Status", "turn ");

                    if (!robotMoving) {
                        robotMove(5000,0);
                        robotMoving = true;
                    }

                    if (!moveComplete()) {
                        telemetry.addData("Position:", "Left: " + motorFrontLeft.getCurrentPosition() + " Right: " + motorFrontRight.getCurrentPosition());
                    } else {
                        //initialODSReading = opticalDistanceSensor.getLightDetected();
                        telemetry.addData("SucceededPosition", "Left: " + motorFrontLeft.getCurrentPosition() + " Right: " + motorFrontRight.getCurrentPosition());
                        telemetry.addData("final ", "ODS:  " + String.format("%.2f", initialODSReading));
                        robotMoving = false;
                        step++;
                        autoShutdown();
                    }
                    initialODSReading = opticalDistanceSensor.getLightDetected();
                    telemetry.addData("5 ", "ODS:  " + String.format("%.2f", initialODSReading));
                    break;

                case 3:
                    //Rotate the robot to position the shelter arm ready to drop in to the basket
                    telemetry.addData("Status", "Dump the little guys inside basket");

                    if (!robotMoving) {
                        //shelter.setPosition(0.3);
                        bShelter = true;
                    }

                    if (bShelter) {
                        telemetry.addData("Reset shelter: ", "completed");
                        //shelter.setPosition(0.7);
                    } else {
                        currentODSReading = opticalDistanceSensor.getLightDetected();
                        telemetry.addData("SucceededPosition", "Left: " + motorFrontLeft.getCurrentPosition() + " Right: " + motorFrontRight.getCurrentPosition());
                        telemetry.addData("final ", "ODS:  " + String.format("%.2f", currentODSReading));
                        robotMoving = false;
                        step++;

                        autoShutdown();
                        //shelter.setPosition(0.5);

                    }
                    currentODSReading = opticalDistanceSensor.getLightDetected();
                    telemetry.addData("6 ", "ODS:  " + String.format("%.2f", currentODSReading));
                    double diff = Math.abs (initialODSReading - currentODSReading);
                    telemetry.addData("7", "diff : " + String.format("%.2f", diff));

                    if (diff>= 0.01 && diff <=0.19) {
                        telemetry.addData("different ", "ODS:  " + String.format("%.2f", currentODSReading));
                        motorBackLeft.setPower(-0.3);
                        motorBackRight.setPower(0.3);
                        motorFrontLeft.setPower(-0.3);
                        motorFrontRight.setPower(0.3);
                    } else {
                        motorBackLeft.setPower(0.0);
                        motorBackRight.setPower(0.0);
                        motorFrontRight.setPower(0.0);
                        motorFrontRight.setPower(0.0);
                    }
                    break;

                default:
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