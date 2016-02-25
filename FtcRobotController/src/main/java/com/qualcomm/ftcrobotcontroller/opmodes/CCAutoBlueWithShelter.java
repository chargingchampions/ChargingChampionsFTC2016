/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.lang.reflect.Constructor;

//------------------------------------------------------------------------------
// CCAutonomousBlue.java
//------------------------------------------------------------------------------
// Extends the OpMode class to provide a Example Autonomous code
//------------------------------------------------------------------------------
/* This opMode does the following steps:
 * 0) Wait till the encoders show reset to zero.
 * 1) Drives to the vicinity of the beacon using encoder counts
 * 2) Use the Legacy light sensor to locate the white line
 * 3) Tracks the line until the wall is reached
 * 4) Pushes up against wall to get square using constant power an time.
 * 5) Deploys the Climbers using the servo
 * 6) Drives to the Mountain using encoder counts
 * 7) Climbs the Mountain using constant speed and time
 * 8) Stops and waits for end of Auto
 *
 * The code is executed as a state machine.  Each "State" performs a specific task which takes time to execute.
 * An "Event" can cause a change in the state.  One or more "Actions" are performed when moving on to next state
 */

public class CCAutoBlueWithShelter extends OpMode
{
    // A list of system States.
    private enum State
    {
        STATE_INITIAL,
        STATE_DRIVE_TO_BEACON,
        STATE_LOCATE_LINE,
        STATE_FOLLOW_LINE,
        STATE_SQUARE_TO_WALL,
        STATE_DEPLOY_CLIMBERS,
        STATE_DRIVE_TO_MOUNTAIN,
        STATE_CLIMB_MOUNTAIN,
        STATE_STOP,
    }

    // Define driving paths as pairs of relative wheel movements in inches (left,right) plus speed %
    final PathSeg[] mBeaconPath = {
            new PathSeg( 19.5, 19.5, 0.5),  // Forward
            new PathSeg( -22.5, 9.0, 0.2),  // Right
            new PathSeg( 30.0, 30.0, 0.5),  // Forward
    };

    final PathSeg[] mMountainPath = {
            new PathSeg( 0.0, 0.0, 0.0),  // Left Rev
            new PathSeg( 0.0, 0.0, 0.0),  // Backup
            new PathSeg( 0.0, 0.0, 0.0),  // Right Rev
            new PathSeg( 0.0, 0.0, 0.0),  // Forward
    };

    final double COUNTS_PER_INCH = 240 ;    // Number of encoder counts per inch of wheel travel.

    final double WHITE_THRESHOLD = 0.5 ;
    final double RANGE_THRESHOLD = 0.5 ;
    final double CLIMBER_RETRACT = 0.7 ;
    final double CLIMBER_DEPLOY  = 0.5 ;
    boolean firstTenSecondsDone=false;

    //--------------------------------------------------------------------------
    // Robot device Objects
    //--------------------------------------------------------------------------
    double odsInitialValue;

    // OpticalDistanceSensor to locate the white line
    // near the beacon repair zone.
    OpticalDistanceSensor mDistance;

    // Initialize left and right drive motors
    DcMotor motorBackRight, motorFrontRight;
    DcMotor motorBackLeft, motorFrontLeft;

    // Motors for Hook and Pulley
    DcMotor motorHookRight, motorHookLeft, motorPulley;

    //Servos for linear Slider Tilt
    Servo linearSliderRight, linearSliderLeft;

    //Servos for ziplines
    Servo zipLineRight, zipLineLeft;

    //Servo for shelter
    Servo shelter;

    //Servo for ball sweep
    Servo ballSweep;

    private int         mLeftEncoderTarget;
    private int         mRightEncoderTarget;

    // Loop cycle time stats variables
    public ElapsedTime  mRuntime = new ElapsedTime();   // Time into round.

    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state
    private double  startTime;

    private State       mCurrentState;    // Current State Machine State.
    private PathSeg[]   mCurrentPath;     // Array to hold current path
    private int         mCurrentSeg;      // Index of the current leg in the current path


    //--------------------------------------------------------------------------
    // Constructor
    //--------------------------------------------------------------------------
    public CCAutoBlueWithShelter()
    {
    }

    //--------------------------------------------------------------------------
    // init
    //--------------------------------------------------------------------------
    @Override
    public void init()
    {
        // Initialize class members.
        motorBackLeft = hardwareMap.dcMotor.get("back_left");
        motorBackRight = hardwareMap.dcMotor.get("back_right");
        motorFrontLeft = hardwareMap.dcMotor.get("front_left");
        motorFrontRight = hardwareMap.dcMotor.get("front_right");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        mDistance = hardwareMap.opticalDistanceSensor.get("sensor_ods");
        odsInitialValue = mDistance.getLightDetected();

        motorHookLeft = hardwareMap.dcMotor.get("hook_left");
        motorHookRight = hardwareMap.dcMotor.get("hook_right");
        motorPulley = hardwareMap.dcMotor.get("pulley");

        linearSliderRight = hardwareMap.servo.get("linear_right");
        linearSliderLeft = hardwareMap.servo.get("linear_left");
        zipLineRight = hardwareMap.servo.get("zipline_right");
        zipLineLeft = hardwareMap.servo.get("zipline_left");
        shelter = hardwareMap.servo.get("shelter");
        ballSweep = hardwareMap.servo.get("ball_sweep");

        shelter.setPosition(0.5);
        ballSweep.setPosition(0.5);
        zipLineLeft.setPosition(0.5);
        zipLineRight.setPosition(0.5);
        linearSliderLeft.setPosition(0.5);
        linearSliderRight.setPosition(0.5);

        motorHookLeft.setPower(0.0);
        motorHookRight.setPower(0.0);
        motorPulley.setPower(0.0);

        motorBackLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorFrontRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        motorBackLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBackRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFrontLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorFrontRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        odsInitialValue=mDistance.getLightDetected();
        telemetry.addData("ODSInitial: ", odsInitialValue);


        setDrivePower(0, 0);        // Ensure motors are off
        resetDriveEncoders();       // Reset Encoders to Zero
    }

    //--------------------------------------------------------------------------
    // loop
    //--------------------------------------------------------------------------
    // @Override
    public void init_loop()
    {
        // Keep resetting encoders and show the current values
        resetDriveEncoders();        // Reset Encoders to Zero
        telemetry.addData("ENC Front", String.format("L:R %d:%d", getFrontLeftPosition(), getFrontRightPosition()));
        telemetry.addData("ENC Back", String.format("L:R %d:%d", getBackLeftPosition(), getBackRightPosition()));
    }

    //--------------------------------------------------------------------------
    // start
    //--------------------------------------------------------------------------
    @Override
    public void start()
    {
        // Setup Robot devices, set initial state and start game clock
        setDriveSpeed(0, 0);        // Set target speed to zero
        runToPosition();            // Run to Position set by encoder targets
        mRuntime.reset();           // Zero game clock
        startTime = getRuntime();
        newState(State.STATE_INITIAL);
    }

    //--------------------------------------------------------------------------
    // loop
    //--------------------------------------------------------------------------
    @Override
    public void loop()
    {
        hardwareMap.logDevices();
        if(!firstTenSecondsDone && startTime + 8 <= getRuntime())
            firstTenSecondsDone = true;
        // Send the current state info (state and time) back to first line of driver station telemetry.
        telemetry.addData("0", String.format("%4.1f ", mStateTime.time()) + mCurrentState.toString());

        // Execute the current state.  Each STATE's case code does the following:
        // 1: Look for an EVENT that will cause a STATE change
        // 2: If an EVENT is found, take any required ACTION, and then set the next STATE
        //   else
        // 3: If no EVENT is found, do processing for the current STATE and send TELEMETRY data for STATE.
        //
        if (firstTenSecondsDone) {
            switch (mCurrentState) {
                case STATE_INITIAL:         // Stay in this state until encoders are both Zero.
                    if (encodersAtZero()) {
                        startPath(mBeaconPath);                 // Action: Load path to beacon
                        newState(State.STATE_DRIVE_TO_BEACON);  // Next State:
                        telemetry.addData("Autonomous", "here!");
                    } else {
                        // Display Diagnostic data for this state.
                        telemetry.addData("1", String.format("L %5d - R %5d ", getFrontLeftPosition(),
                                getFrontRightPosition()));
                        telemetry.addData("Autonomous", "here1!");
                    }

                    break;

                case STATE_DRIVE_TO_BEACON: // Follow path until last segment is completed
                    if (pathComplete()) {
                        //mLight.enableLed(true);                 // Action: Enable Light Sensor
                        setDriveSpeed(-0.1, 0.1);               // Action: Start rotating left
                        newState(State.STATE_LOCATE_LINE);      // Next State:
                        telemetry.addData("Autonomous", "here2!");
                    } else {
                        telemetry.addData("Autonomous", "here2");
                        // Display Diagnostic data for this state.
                        telemetry.addData("1", String.format("%d of %d. L %5d:%5s - R %5d:%5d ",
                                mCurrentSeg, mCurrentPath.length,
                                mLeftEncoderTarget, getFrontLeftPosition(),
                                mRightEncoderTarget, getFrontRightPosition()));
                    }
                    break;

                case STATE_LOCATE_LINE:     // Rotate until white tape is detected
                    newState(State.STATE_FOLLOW_LINE);
                    telemetry.addData("Autonomous", "here3");
                  /*if (mLight.getLightDetected() > WHITE_THRESHOLD)
                    {
                        setDriveSpeed(0.0, 0.0);                // Action: Stop rotation
                        newState(State.STATE_FOLLOW_LINE);      // Next State:
                    }
                    else
                    {
                        // Display Diagnostic data for this state.
                        telemetry.addData("1", String.format("%4.2f of %4.2f ",
                                mLight.getLightDetected(),
                                WHITE_THRESHOLD ));
                    }*/

                    break;

                case STATE_FOLLOW_LINE:     // Track line until wall is reached
                    newState(State.STATE_SQUARE_TO_WALL);
                    telemetry.addData("Autonomous", "here4");

                    useConstantPower();                     // Apply constant power to motors
                    setDriveSpeed(0.1, 0.1);                // Action: Drive Forward
                    newState(State.STATE_SQUARE_TO_WALL);   // Next State:

                    /*else
                    {
                        // Steer left ot right
                        if (mLight.getLightDetected() > WHITE_THRESHOLD)
                        {
                            setDriveSpeed(0.2, 0.0);            // Scan Right
                            telemetry.addData("1", String.format("%4.2f --> %7d : %7d (%4.2f)" ,
                                    mLight.getLightDetected(),
                                    getLeftPosition(), getRightPosition(),
                                    mDistance.getLightDetected() ));
                        }
                        else
                        {
                            setDriveSpeed(0.0, 0.2);            // Scan Left
                            telemetry.addData("1", String.format("%4.2f <-- %7d : %7d (%4.2f)",
                                    mLight.getLightDetected(),
                                    getLeftPosition(), getRightPosition(),
                                    mDistance.getLightDetected() ));
                        }
                    } */
                    break;

                case STATE_SQUARE_TO_WALL:     // Push up against wall for 1 second
                    if (mStateTime.time() > 1.0) {
                        DbgLog.msg("In State_Square_To_wall");
                        telemetry.addData("Autonomous", ">1!");
                        setDriveSpeed(0.0, 0.0);                // Action: Stop pushing
                        double odsSecondValue = mDistance.getLightDetected();

                        //if ((odsInitialValue * 4) < odsSecondValue) {
                            shelter.setPosition(0.3);     // Action:  Start deploying climbers.
                      //  } else
                            telemetry.addData("odsSecondValue: ", odsSecondValue);
                        newState(State.STATE_DEPLOY_CLIMBERS);  // Next State:
                    } else
                        telemetry.addData("Autonomous", "<1!");
                    break;

                case STATE_DEPLOY_CLIMBERS:     // wait 2 seconds while servos move and deposit climbers
                    if (mStateTime.time() > 4.0) {
                        telemetry.addData("Autonomous", ">2!");
                        shelter.setPosition(0.7);    // Put servo into "starting position"
                        //startPath(mMountainPath);               // Action: Load path to Mountain
                        //newState(State.STATE_DRIVE_TO_MOUNTAIN);// Next State:
                        newState(State.STATE_STOP);// Next State:
                    } else
                        telemetry.addData("Autonomous", "<2!");
                    break;

              /*  case STATE_DRIVE_TO_MOUNTAIN: // Follow path until last segment is completed
                    if (pathComplete())
                    {
                        useConstantPower();                     // Action: Switch to constant Speed
                       // setDrivePower(0.5, 0.5);                // Action: Start Driving forward at 50 Speed
                        newState(State.STATE_CLIMB_MOUNTAIN);   // Next State:
                    }
                    else
                    {
                        // Display Diagnostic data for this state.
                        telemetry.addData("1", String.format("%d of %d. L %5d:%5d - R %5d:%5d ",
                                mCurrentSeg, mCurrentPath.length,
                                mLeftEncoderTarget, getFrontLeftPosition(),
                                mRightEncoderTarget, getFrontRightPosition()));
                    }
                    break;

                case STATE_CLIMB_MOUNTAIN:   // Drive up mountain for 5 seconds
                    if (mStateTime.time() > 5.0)
                    {
                        useConstantPower();                     // Switch to constant Power
                        setDrivePower(0, 0);                    // Set target speed to zero
                        newState(State.STATE_STOP);             // Next State:
                    }
                    else
                    {
                        // Display Diagnostic data for this state.
                        telemetry.addData("1", String.format("L %5d - R %5d ", getFrontLeftPosition(),
                                getFrontRightPosition() ));
                    }
                    break; */

                case STATE_STOP:
                    shelter.setPosition(0.5);
                    if (mRuntime.time()> 28.0) {
                        useConstantPower();
                        setDrivePower(0, 0);
                        shelter.setPosition(0.5);
                        ballSweep.setPosition(0.5);
                        zipLineLeft.setPosition(0.5);
                        zipLineRight.setPosition(0.5);
                        linearSliderLeft.setPosition(0.5);
                        linearSliderRight.setPosition(0.5);

                        motorHookLeft.setPower(0.0);
                        motorHookRight.setPower(0.0);
                        motorPulley.setPower(0.0);
                        telemetry.addData("Stop all motors and servos ", mRuntime.toString());
                    }
                    break;
            }
        }
    }

    //--------------------------------------------------------------------------
    // stop
    //--------------------------------------------------------------------------
    @Override
    public void stop()
    {
        // Ensure that the motors are turned off.
        useConstantPower();
        setDrivePower(0, 0);
    }

    //--------------------------------------------------------------------------
    // User Defined Utility functions here....
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    //  Transition to a new state.
    //--------------------------------------------------------------------------
    private void newState(State newState)
    {
        // Reset the state time, and then change to next state.
        mStateTime.reset();
        mCurrentState = newState;
    }


    //--------------------------------------------------------------------------
    // setEncoderTarget( LeftEncoder, RightEncoder);
    // Sets Absolute Encoder Position
    //--------------------------------------------------------------------------
    void setEncoderTarget(int leftEncoder, int rightEncoder)
    {
        motorBackLeft.setTargetPosition(mLeftEncoderTarget = leftEncoder);
        motorFrontLeft.setTargetPosition(mLeftEncoderTarget = leftEncoder);
        motorBackRight.setTargetPosition(mRightEncoderTarget = rightEncoder);
        motorFrontRight.setTargetPosition(mRightEncoderTarget = rightEncoder);
    }

    //--------------------------------------------------------------------------
    // addEncoderTarget( LeftEncoder, RightEncoder);
    // Sets relative Encoder Position.  Offset current targets with passed data
    //--------------------------------------------------------------------------
    void addEncoderTarget(int leftEncoder, int rightEncoder)
    {
        motorBackLeft.setTargetPosition(mLeftEncoderTarget += leftEncoder);
        motorFrontLeft.setTargetPosition(mLeftEncoderTarget += leftEncoder);
        motorBackRight.setTargetPosition(mRightEncoderTarget += rightEncoder);
        motorFrontRight.setTargetPosition(mRightEncoderTarget += rightEncoder);
    }

    //--------------------------------------------------------------------------
    // setDrivePower( LeftPower, RightPower);
    //--------------------------------------------------------------------------
    void setDrivePower(double leftPower, double rightPower)
    {
        motorBackLeft.setPower(Range.clip(leftPower, -1, 1));
        motorBackRight.setPower(Range.clip(rightPower, -1, 1));
        motorFrontLeft.setPower(Range.clip(leftPower, -1, 1));
        motorFrontRight.setPower(Range.clip(rightPower, -1, 1));
    }

    //--------------------------------------------------------------------------
    // setDriveSpeed( LeftSpeed, RightSpeed);
    //--------------------------------------------------------------------------
    void setDriveSpeed(double leftSpeed, double rightSpeed)
    {
        setDrivePower(leftSpeed, rightSpeed);
    }

    //--------------------------------------------------------------------------
    // runToPosition ()
    // Set both drive motors to encoder servo mode (requires encoders)
    //--------------------------------------------------------------------------
    public void runToPosition()
    {
        setDriveMode(DcMotorController.RunMode.RUN_TO_POSITION);
    }

    //--------------------------------------------------------------------------
    // useConstantSpeed ()
    // Set both drive motors to constant speed (requires encoders)
    //--------------------------------------------------------------------------
    public void useConstantSpeed()
    {
        setDriveMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    //--------------------------------------------------------------------------
    // useConstantPower ()
    // Set both drive motors to constant power (encoders NOT required)
    //--------------------------------------------------------------------------
    public void useConstantPower()
    {
        setDriveMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

    //--------------------------------------------------------------------------
    // resetDriveEncoders()
    // Reset both drive motor encoders, and clear current encoder targets.
    //--------------------------------------------------------------------------
    public void resetDriveEncoders()
    {
        setEncoderTarget(0, 0);
        setDriveMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    //--------------------------------------------------------------------------
    // syncEncoders()
    // Load the current encoder values into the Target Values
    // Essentially synch's the software with the hardware
    //--------------------------------------------------------------------------
    void synchEncoders()
    {
        //	get and set the encoder targets
        mLeftEncoderTarget = motorFrontLeft.getCurrentPosition();
        mRightEncoderTarget = motorFrontRight.getCurrentPosition();
    }

    //--------------------------------------------------------------------------
    // setDriveMode ()a
    // Set both drive motors to new mode if they need changing.
    //--------------------------------------------------------------------------
    public void setDriveMode(DcMotorController.RunMode mode)
    {
        // Ensure the motors are in the correct mode.
        if (motorBackLeft.getMode() != mode) {
            motorBackLeft.setMode(mode);
            motorBackRight.setMode(mode);
        }

        if (motorFrontLeft.getMode() != mode) {
            motorFrontLeft.setMode(mode);
            motorFrontRight.setMode(mode);
        }

    }

    //--------------------------------------------------------------------------
    // getLeftPosition ()
    // Return Left Encoder count
    //--------------------------------------------------------------------------
    int getBackLeftPosition()
    {
        return motorBackLeft.getCurrentPosition();
    }

    //--------------------------------------------------------------------------
    // getRightPosition ()
    // Return Right Encoder count
    //--------------------------------------------------------------------------
    int getBackRightPosition()
    {
        return motorBackRight.getCurrentPosition();
    }

    int getFrontLeftPosition()
    {
        return motorFrontLeft.getCurrentPosition();
    }

    //--------------------------------------------------------------------------
    // getRightPosition ()
    // Return Right Encoder count
    //--------------------------------------------------------------------------
    int getFrontRightPosition()
    {
        return motorFrontRight.getCurrentPosition();
    }

    //--------------------------------------------------------------------------
    // moveComplete()
    // Return true if motors have both reached the desired encoder target
    //--------------------------------------------------------------------------
    boolean moveComplete()
    {
        //  return (!mLeftMotor.isBusy() && !mRightMotor.isBusy());
        telemetry.addData("motor pos: ", motorBackLeft.getCurrentPosition());
        return ((Math.abs(getBackLeftPosition() - mLeftEncoderTarget) < 10) ||
                (Math.abs(getBackRightPosition() - mRightEncoderTarget) < 10) ||
                (Math.abs(getFrontLeftPosition() - mLeftEncoderTarget) < 10) ||
                (Math.abs(getFrontRightPosition() - mRightEncoderTarget) < 10));
    }

    //--------------------------------------------------------------------------
    // encodersAtZero()
    // Return true if both encoders read zero (or close)
    //--------------------------------------------------------------------------
    boolean encodersAtZero()
    {
        return ((Math.abs(getFrontLeftPosition()) < 5) && (Math.abs(getFrontRightPosition()) < 5) &&
                (Math.abs(getBackLeftPosition()) < 5) && (Math.abs(getBackRightPosition()) < 5) );
    }

    /*
        Begin the first leg of the path array that is passed in.
        Calls startSeg() to actually load the encoder targets.
     */
    private void startPath(PathSeg[] path)
    {
        mCurrentPath = path;    // Initialize path array
        mCurrentSeg = 0;
        synchEncoders();        // Lock in the current position
        runToPosition();        // Enable RunToPosition mode
        startSeg();             // Execute the current (first) Leg
    }

    /*
        Starts the current leg of the current path.
        Must call startPath() once before calling this
        Each leg adds the new relative movement onto the running encoder totals.
        By not reading and using the actual encoder values, this avoids accumulating errors.
        Increments the leg number after loading the current encoder targets
     */
    private void startSeg()
    {
        int Left;
        int Right;

        if (mCurrentPath != null)
        {
            // Load up the next motion based on the current segemnt.
            Left  = (int)(mCurrentPath[mCurrentSeg].mLeft * COUNTS_PER_INCH);
            Right = (int)(mCurrentPath[mCurrentSeg].mRight * COUNTS_PER_INCH);
            DbgLog.msg("setting encoders, left: "+ Left+" Right: "+Right);
            addEncoderTarget(Left, Right);
            setDriveSpeed(mCurrentPath[mCurrentSeg].mSpeed, mCurrentPath[mCurrentSeg].mSpeed);

            telemetry.addData("mCurrentSeg", mCurrentSeg);
            telemetry.addData("mSpeed", mCurrentPath[mCurrentSeg].mSpeed);

            mCurrentSeg++;  // Move index to next segment of path
        }
    }

    /*
        Determines if the current path is complete
        As each segment completes, the next segment is started unless there are no more.
        Returns true if the last leg has completed and the robot is stopped.
     */
    private boolean pathComplete()
    {
        // Wait for this Segement to end and then see what's next.
        if (moveComplete())
        {
            telemetry.addData("pathCmplete", "moveComplete true!");
            // Start next Segement if there is one.
            if (mCurrentSeg < mCurrentPath.length)
            {
                startSeg();
            }
            else  // Otherwise, stop and return done
            {
                telemetry.addData("pathCmplete", "currentsegment<mcurrentpath.lengthj false!");
                mCurrentPath = null;
                mCurrentSeg = 0;
                setDriveSpeed(0, 0);
                useConstantSpeed();
                return true;
            }
        } else
            telemetry.addData("pathCmplete", "moveComplete false!");
        return false;
    }
}
