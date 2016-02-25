package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by rneervannan on 2/14/2016.
 */

/**
 * Define a "PathSegment" object, used for building a path for the robot to follow.
 */
class PathSeg
{
    public double mLeft;
    public double mRight;
    public double mSpeed;

    // Constructor
    public PathSeg(double Left, double Right, double Speed)
    {
        mLeft = Left;
        mRight = Right;
        mSpeed = Speed;
    }
}