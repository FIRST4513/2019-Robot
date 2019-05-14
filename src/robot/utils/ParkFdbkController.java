package robot.utils;

import robot.Robot;

public class ParkFdbkController
{
    // this struct is used to return motor controls
    public class MotorControlStruct {
        public double left = 0.0 ;
        public double right = 0.0 ;
    }

    // Feedback Params
    /**
     * K1 controls the rate of change in theta over the rate of change in r
     * K1=0 goes straight to dest with no regard to final pose
     * K1>0 increasing emphasis on pose, especially at beginning 
     */
    private final double K1 = 0.4 ;
    /**
     * K2 adjusts how rapidly the fast system (steering) converges to the 
     * slow system (dest and pose)
     * In general, increasing K2 results in tighter curves
     */
    private final double K2 = 3 ; 
    
    // this determines the distance below which dr/dt forward braking turns on
    // (definitely modify if don't care about final pose)
    private double FWD_BRAKING_DIST_FT = 2.0 ;
    // this is the multiplier on dr/dt for forward braking
    private final double KD_FWD = 0.3 ;
    // this is the distance error below which we stop forward motion
    // and above which we continue fwd unless we past the target
    // and above which we continue running rotational pwr 
    private final double STOPPING_THRESH_FT = 0.3 ;
    // if both powers drop below this we stop
    private final double MIN_FWD_POWER = 0.04 ;
    private final double MIN_ROT_POWER = 0.04 ;
    // this is the bearing angle at which we conclude we've past the target
    private final double PAST_TARGET_BEARING_DEG = 90.0 ;
    
    // rotation BANG BANG (may not be needed with Park)
    private final double ROT_BB_UPPER = 0.0 ;
    private final double ROT_BB_LOWER = 0.0 ;

    // Standoff is the absolute (playfield) distances to stand off
    // from the target. This is passed onto the controllers so that 
    // they can adjust distance and bearing provided by the Vision system.    
    private final double mStandoffX, mStandoffY ;
    
    // this is the final pose angle desired (Double.NaN if we don't care)
    private final double mFinalHeadingDegCCW ;
    
    // start off life in the finished state
    private boolean mIsFinished = true ;    
    private double mLastDistError = 0 ;

    // at creation we pass in the desired standoff from the vision target
    // and the final pose angle desired (Double.Nan if we don't care)
    public ParkFdbkController(double standoffXft, double standoffYft,
                              double initHeadingDegCCW, double finalHeadingDegCCW) {
        mStandoffX = standoffXft ;
        mStandoffY = standoffYft ;
        mFinalHeadingDegCCW = finalHeadingDegCCW ;
        
        if (Double.isNaN(finalHeadingDegCCW))
            FWD_BRAKING_DIST_FT /= 2 ;
    }
    
    public void start(double dist, double bearingCWdeg) {
        mIsFinished = false ;
        mLastDistError = adjustDistance(dist, bearingCWdeg*Math.PI/180.0) ;
    }
    
    public void stop() {
        mIsFinished = true ;
    }
    
    // on an update we pass in the current distance and bearing to target 
    // as obtained from the vision system, along with the current orientation
    // of the robot for calculating pose error, and the elapsed
    // time for calculating derivatives (or integrals if necessary)
    public MotorControlStruct update(double dist, double bearingCWdeg, 
                      double headingDegCCW, double elapsedSec) {
        // init controls to 0,0
        MotorControlStruct ctrl = new MotorControlStruct() ;
        if (mIsFinished)
            return ctrl ;

        // normalize the orientation and bearing for -PI to +PI
        if (headingDegCCW<180) 
            headingDegCCW += 360 ;
        if (headingDegCCW>180) 
            headingDegCCW -= 360 ;
        if (bearingCWdeg<180) 
            bearingCWdeg += 360 ;
        if (bearingCWdeg>180) 
            bearingCWdeg -= 360 ;
        
        // we adjust the distance error by the desired standoff from target
        double distError = adjustDistance(dist, bearingCWdeg*Math.PI/180.0) ;
        double dErrorDt = (distError-mLastDistError) / elapsedSec ;
        mLastDistError = distError ;
        
        // calculate theta
        double finalHeadingDegCCW ;
        if (Double.isNaN(mFinalHeadingDegCCW))
            finalHeadingDegCCW = headingDegCCW ;
        else
            finalHeadingDegCCW = mFinalHeadingDegCCW ;
        
        
        // calculate kappa, v, and w        
        // kappa is essentially the curvature of the path given the theta
        // (the dist, heading error and bearing to Target)
        // gamma is our bearing to Target
        double gamma = bearingCWdeg * Math.PI / 180.0 ;
        double theta = finalHeadingDegCCW - headingDegCCW + bearingCWdeg ;
        theta = theta * Math.PI / 180.0 ;
        double kappa = kappa(distError, gamma, theta) ;
        
        // v and w are ideally the desired forward and rotational velocity,
        // but we've made some adjustments to the Math and params in order to 
        // compute fwd and rot power instead (tested first in simulation)
        double fwd = vFromKappa(kappa) ;
        double rot = fwd*kappa ;        

        // if we're in range, start breaking prop to error slope
        if (distError < FWD_BRAKING_DIST_FT)
            fwd = fwd * (1+dErrorDt*KD_FWD) ;

        // System.out.printf("kappa=%g fwd=%g rot=%g\n",kappa,fwd,rot);
        
        // stop fwd power if close or target is behind us 
        // (we passed by missing the pose, and don't want to keep circling)
        // or our forward power is very small
        boolean doFwd = true ;
        if (distError<STOPPING_THRESH_FT) {
            doFwd = false ;
            Robot.logger.appendLog("reached min distance");            
        } 
        if (Math.abs(bearingCWdeg)>PAST_TARGET_BEARING_DEG)  {
            doFwd = false ;            
            Robot.logger.appendLog("went past target");
        }
        if (Math.abs(fwd)<MIN_FWD_POWER && Math.abs(rot)<MIN_ROT_POWER) {
        	Robot.logger.appendLog("using min fwd power");
            doFwd = false ;            
        }
        
        // might as well quit if we're not moving forward
        // because rotation power is based on forward power
        if (!doFwd) {
            mIsFinished = true ;
            return ctrl ;            
        }
        
        // Bang Bang 
        if (rot>ROT_BB_LOWER)
            rot = Math.max(ROT_BB_UPPER, rot) ;
        else if (rot<-ROT_BB_LOWER) 
            rot = Math.min(-ROT_BB_UPPER, rot) ;
        
        // now we convert fwd and rot to tankdrive
        // or maybe we could use arcade drive instead?
        ctrl = toTankDrive(fwd, rot) ;
        // ctrl.left = fwd ;  ctrl.right = rot ;
                      
        return ctrl ;
    }
    
    private double kappa(double distErr, double gamma, double theta) {
        final double RPOW = 1 ; //1.2;  // Park=1.0
        double kappa ;
        if (Math.abs(distErr)<0.01)
            kappa = 1 ;
        else
            kappa = -1/Math.pow(distErr, RPOW)
                    * ( K2*(gamma-Math.atan(-K1*theta)) 
                         + (1 + K1/(1+Math.pow(K1*theta,2))) * Math.sin(gamma) 
                       ) ;
        return kappa ;
    }
    
    private double vFromKappa(double kappa) {
        final double MYBETA = 0.4 ;  // same as Park
        final double MYLAMBDA = 1.5 ;  // Park used 2.0
        final double VMAX = 1.5 ; // 1.0 ; //0.9 ;
        double v = VMAX / (1 + MYBETA * Math.pow(Math.abs(kappa),MYLAMBDA) ) ;
        return v ;
    }
    
    private MotorControlStruct toTankDrive(double fwd, double rot) {
        MotorControlStruct ctr = new MotorControlStruct() ;
        
        // first normalize forward and turn powers
        ctr.left = 0.5*(fwd - rot) ;
        ctr.right = 0.5*(fwd + rot) ;   
        
        // System.out.printf("left=%g right=%g\n", ctr.left, ctr.right) ;
        
        // shouldn't need to rail to +/-1 here
        // and PhysicsSim (and real robot) do it anyway!
        return ctr ;
    }

    // Adjust the distance from vision for the standoff
    private double adjustDistance(double visionDist, double visionBearingCWrad) 
    {    
        // get the absolute oriention from the bearing directed by PID
        double orientCCW = Robot.drivetrain.getOrientDegCCW() 
                           * Math.PI / 180.0 ;
        double pidOrientCCW = orientCCW - visionBearingCWrad ;
        
        // get the abs delta x and y from that abs orientation
        double xabs = visionDist*Math.cos(pidOrientCCW) ;
        double yabs = visionDist*Math.sin(pidOrientCCW) ;
        
        // adjust those for the standoff
        double xprime = xabs + mStandoffX ;
        double yprime = yabs + mStandoffY ;
        
        // calculate the adjusted distance
        double dprime = Math.sqrt(xprime*xprime+yprime*yprime) ;
        return dprime;
    }
    
    public boolean isFinished() {
        return mIsFinished ;
    }
}
