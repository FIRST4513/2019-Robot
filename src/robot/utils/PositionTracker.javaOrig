package robot.utils;

import robot.subsystems.Drivetrain;

/***
 * To use:
 * 0) Paste this file into your project into any package
 * 1) Add the necessary imports above (IDE should prompt after the paste)
 * 2) Make DriveTrain.getLeftEncoderDist() and getRightEncoderDist public
 * 3) Instantiate a PositionTracker, giving it the required parameters
 * 4) Before movement starts, reset your encoders and call init() on it
 * 5) Every 20 msec or so, call updatePositions(), which will return an
 *    object of type PositionTracker.Posn with three public member vars:
 *    x, y, yaw (in radians, with pos being a CCW orientation).
 * 6) If radians and CCW are inconvenient, call getPosition(), which will 
 *    return the current position with positive yaw in degrees CW.
 * Note that the returned positions are relative to the position when init() 
 * was called. So they accumulate over time until you call init() again.
 */

public class PositionTracker 
{
    public class Posn {
        public double x ;
        public double y ;
        public double yaw ;
        public Posn() {
            x = 0 ; y = 0 ; yaw = 0 ;
        }
        public Posn(Posn p) {
            x = p.x ; y=p.y ; yaw = p.yaw ;
        }
    } ;
    
    Drivetrain mDriveTrain ;
    double     mLeftDist, mRightDist ;
    double     mWheelBase ;
    Posn       mPosn ; ;

    public PositionTracker(Drivetrain drivetrain, double wheelBase) {
        mDriveTrain = drivetrain;
        mWheelBase = wheelBase ;
        mPosn = new Posn() ;
    }
    
    public void init(boolean resetOrigin) {
        if (resetOrigin) {
           mPosn = new Posn() ;            
        }
        mLeftDist = 0 ;
        mRightDist = 0 ;
    }

    public Posn updatePositions() {
        double left = mDriveTrain.getLeftDist() ;
        double right = mDriveTrain.getRightDist() ;
        // calculate change in position
        double leftDist = left - mLeftDist ;
        double rightDist = right - mRightDist ;
        // update previous position
        mLeftDist = left ;
        mRightDist = right ;

        // no restriction on wheel traveling the same direction
        // sum and difference between the two wheels
        double rightMinusLeft = rightDist - leftDist;
        double rightPlusLeft = rightDist + leftDist;

        // Note that, for forward travel,
        // a left turn will result in a positive angle and a positive radius
        // a right turn will result in a negative angle and a negative radius
        double angle = rightMinusLeft / mWheelBase ;

        if (angle != 0) {
            // movement in local coordinate system of the robot
            double radius = (rightPlusLeft) / (2*angle) ;
            double arcseg = Math.sqrt(2*radius*radius*(1-Math.cos(angle))) ;
            double angleprime ; 
            if (radius == 0) 
                angleprime = 0 ;
            else 
                angleprime = Math.acos(arcseg/2/radius) ;
                
            double rely = (radius * Math.sin(angle)) ;
            double relx = (arcseg * Math.cos(angleprime)) ;
            
            // convert those to global coordinate system via rotation
            mPosn.yaw += angle ;
            normalizeYaw() ;
            double cos = Math.cos(mPosn.yaw) ;
            double sin = Math.sin(mPosn.yaw) ;
            double x = cos*relx - sin*rely ;
            double y = sin*relx + cos*rely ;
            
            // now accumulate
            mPosn.x += x ;
            mPosn.y += y ;
        }
        else {  
            // went straight: use either wheel or the average
            mPosn.y += rightPlusLeft/2.0 ;
            mPosn.x += 0 ;
            mPosn.yaw += 0 ;
        }

        return mPosn ;
    }
    
    public Posn getPosition() {
        Posn posn = new Posn(mPosn) ;
        posn.yaw = -posn.yaw*180/Math.PI ;
        return posn ;
    }
    
    private void normalizeYaw() {
        if (mPosn.yaw < -Math.PI)
            mPosn.yaw += 2*Math.PI ;
        else if (mPosn.yaw > Math.PI)
            mPosn.yaw -= 2*Math.PI ;
    }
}
