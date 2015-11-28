package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
/**
 * Created by rneervannan1 on 11/17/2015.
 */
public class AutonomousOp extends LinearOpMode {
    //left and right motor objects
    DcMotor left = null;
    DcMotor right = null;
    int gearReduction = 1;
    DcMotorController control;

    public void runOpMode() throws InterruptedException {

        EncoderMotorTask motorTask = new EncoderMotorTask( this, left);
        waitForStart();
        int step = 1;
        while (opModeIsActive()) {

            switch (step) {
                case 1:
                    if (! motorTask.isRunning()) {
                        //full power , forward for 8880
                        motorTask.startMotor("step1 motor1", 1, 8880 , Direction.MOTOR_FORWARD);
                    }
                    if (motorTask.targetReached()) {
                        motorTask.stop();
                        step++;
                    }
                    break;

                case 2:
                    if (! motorTask.isRunning()) {
                        //  1/4 power backward for 1000
                        motorTask.startMotor("step2 motor1",0.25, 1000 , Direction.MOTOR_BACKWARD);
                    }
                    if (motorTask.targetReached()) {
                        motorTask.stop();
                        step++;
                    }
                    break;
                default:
                    telemetry.addData("step" + step + " Opmode Status", "Tasks completed");
                    break;
            }
            waitOneFullHardwareCycle();
        }
    }

    void move() {
        while (left.getCurrentPosition() < 100) {
            left.setPower(0.5);
        }
        left.setPower(0.0);
        return;
    }
}



