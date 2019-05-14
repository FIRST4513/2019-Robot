package robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BearingPid
{
    // PID params
    // initial guess control=max at error of 90 deg, KP=1/90= 
    private double KP = 0.01 ; //0.02 for target moving 1 ft/sec ;
    private double KD = 0 ;  
    // we might need bang bang for rotations
    private double BB_UPPER = 0.0 ;
    private double BB_LOWER = 0.0 ;
    
    private double mLastError = 0 ;
    private boolean mIsFinished = true ;
    
    // Standoff is the absolute (playfield) distances to stand off
    // from the target. This is passed onto the controllers so that 
    // they can adjust distance and bearing from the Vision system.    
    private final double mStandoffX, mStandoffY ;
    
    public BearingPid(double standoffXft, double standoffYft) {
        mStandoffX = standoffXft ;
        mStandoffY = standoffYft ;        
    }
    
    public void start(double dist, double bearingToTargCWdeg, 
            		double absOrientCCWdeg) {
        KP = SmartDashboard.getNumber("BEARING_PID_KP", KP) ;
        KD = SmartDashboard.getNumber("BEARING_PID_KD", KD) ;
        BB_UPPER = SmartDashboard.getNumber("BEARING_PID_BB_UPPER", BB_UPPER) ;
        BB_LOWER = SmartDashboard.getNumber("BEARING_PID_BB_LOWER", BB_LOWER) ;
        mIsFinished = false ;
        double bearing = adjustBearing(dist, bearingToTargCWdeg*Math.PI/180.0,
                						absOrientCCWdeg) ;
        mLastError = bearing ;
    }
    
    public void stop() {
        mIsFinished = true ;
    }

    public double update(double distToTarg, double bearingToTargCWdeg, 
            double absOrientCCWdeg, double elapsedSec) {
		if (mIsFinished)
			return 0.0;

		double error = adjustBearing(distToTarg, bearingToTargCWdeg * Math.PI / 180.0, absOrientCCWdeg);
		double dErrorDt = (error - mLastError) / elapsedSec;
		mLastError = error;

		// apply PID
		double control = KP * error + KD * dErrorDt;

		// rail the control to +/- 1
		control = Math.max(control, -1);
		control = Math.min(control, 1);

		// apply BangBang
		if (control > BB_LOWER)
			control = Math.max(BB_UPPER, control);
		else if (control < -BB_LOWER)
			control = Math.min(-BB_UPPER, control);

		return control;
	}
    
    // Adjust the distance from vision for the standoff
   private double adjustBearing(double distToTargFt, double bearingToTargCWrad,
           						double absOrientCCWdeg) {
       // get the absolute oriention from the bearing directed by PID
       double orientCCWrad = absOrientCCWdeg * Math.PI / 180.0 ;
       double targetThetaCCW = orientCCWrad - bearingToTargCWrad ;
       
       // get the abs delta x and y from that abs orientation
       double xabs = distToTargFt*Math.cos(targetThetaCCW) ;
       double yabs = distToTargFt*Math.sin(targetThetaCCW) ;
       
       // adjust those for the standoff
       double xprime = xabs + mStandoffX ;
       double yprime = yabs + mStandoffY ;
       
       // calculate the adjusted bearing
       double adjustedThetaCCWrad = Math.atan(yprime/xprime) ;
       double adjustedBearingToTarg = orientCCWrad - adjustedThetaCCWrad ;
       return adjustedBearingToTarg*180.0/Math.PI;
   }
    
    public boolean isFinished() {
        return mIsFinished ;
    }
    
}
