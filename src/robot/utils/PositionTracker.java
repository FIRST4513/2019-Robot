package robot.utils ;

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
 * was last called.
 */

/**
 * @author paul
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
        // this one takes yaw as CW
        public Posn(double x, double y, double yawDegCW) {
            this.x = x ; this.y=y ; 
            this.yaw = -yawDegCW*Math.PI/180.0 ;
        }
    } ;
       
    // A pointer back to the drivetrain, which must have two methods
    // called getLeftEncodeDist() and getRightEncoderDist. If this is
    // not encapsulated in its own subsystem, it makes more sense for 
    // drivetrain to pass those values into the call to updatePositions().
    // TODO: make that change, or add an option to do it that way.
    Drivetrain mDriveTrain ;

    // This position tracker works from the change in distance since
    // the last update, so the last update is stored here.
    double     mLastLeftDist, mLastRightDist ;
    
    // It needs to know the distance between wheels, in the same units
    // as those provided by the call getLeft and getRightEncoderDist.
    double     mWheelBase ;
    
    // This is the current position, in the same units, since the position
    // tracker was initialized.
    Posn       mPosn ; 
    
    // The current position can be made relative to a starting position
    double     mStartX ;
    double     mStartY ;

    // Constructor: sets 0,0,0 as the initial position
    public PositionTracker(Drivetrain drivetrain, double wheelBase) {
        mDriveTrain = drivetrain ;
        mWheelBase = wheelBase ;
        mPosn = new Posn() ;
    }
    
    // Constructor: pass in the initial position
    public PositionTracker(Drivetrain drivetrain, double wheelBase,
                    double x, double y, double yawDegCW) {
        mDriveTrain = drivetrain;
        mWheelBase = wheelBase ;
        mPosn = new Posn(x, y, yawDegCW) ;
    }
    
    // Initialize the position tracker.
    // Subsequent positions are relative to the point at which this is called.
    public void init() {
        // zero the current position
        mPosn = new Posn() ; 
        // zero the delta distances
        mLastLeftDist = 0 ;
        mLastRightDist = 0 ;
    }

    // Initialize the position tracker.
    // Subsequent positions are relative to the point at which this is called.
    public void init(double x, double y, double yawDegCW) {
        // zero the current position
        mPosn = new Posn(x, y, yawDegCW) ; 
        // zero the delta distances
        mLastLeftDist = 0 ;
        mLastRightDist = 0 ;
    }

    public Posn updatePositions() {
        // Get the current distances from drivetrain.
        // It is assumed that the encoder distances are initialized at the
        // same time as this position tracker (which might be an argument for 
        // managing the encoders from here, or providing a call back so that
        // their distances can be reset from here.
        double left = mDriveTrain.getLeftEncoderDist() ;
        double right = mDriveTrain.getRightEncoderDist() ;
        
        // calculate change in position since the last call
        double leftDeltaDist = left - mLastLeftDist ;
        double rightDeltaDist = right - mLastRightDist ;
        
        // update previous position
        mLastLeftDist = left ;
        mLastRightDist = right ;

        // no restriction on wheel traveling the same direction
        // sum and difference between the two wheels
        double rightMinusLeft = rightDeltaDist - leftDeltaDist;
        double rightPlusLeft = rightDeltaDist + leftDeltaDist;

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
