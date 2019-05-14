package robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.Robot;

public class DistancePid
{
    // PID params
    // inital guess control=max at distance=10 ft, KP=0.1
    private double KP = 2.0 ;
    private double KD = 0.5 ;
    
    private double mLastError = 0 ;
    private boolean mIsFinished = true ;

    // Standoff is the absolute (playfield) distances to stand off
    // from the target. This is passed onto the controllers so that 
    // they can adjust distance and bearing from the Vision system.    
    private final double mStandoffX, mStandoffY ;
    
    public DistancePid(double standoffXft, double standoffYft) {
        mStandoffX = standoffXft ;
        mStandoffY = standoffYft ;
    }
    
    public void start(double dist, double bearingCWdeg) {
        KP = SmartDashboard.getNumber("DISTANCE_PID_KP", KP) ;
        KD = SmartDashboard.getNumber("DISTANCE_PID_KD", KD) ;
        System.out.printf("KP=%g KD=%g\n", KP, KD) ;
        mIsFinished = false ;
        mLastError = adjustDistance(dist, bearingCWdeg*Math.PI/180.0) ;
    }
    
    public void stop() {
        mIsFinished = true ;
    }
    
    public double update(double dist, double bearingCWdeg, double elapsedSec) {
        if (mIsFinished)
            return 0.0 ;
        
        double error = adjustDistance(dist, bearingCWdeg*Math.PI/180.0) ;
        System.out.println("Dist PID Error="+error);
        double dErrorDt = (error-mLastError) / elapsedSec ;
        mLastError = error ;
        
        // stop if we are getting further away
        if (dErrorDt>0 || error<0.25) {
            System.out.printf("dErrorDt=%g error=%g\n", dErrorDt, error);
            System.out.println("Distance PID finished");
            mIsFinished=true ;
        }
        
        double control = KP*error + KD*dErrorDt ;        
        // rail the control to +/- 1
        control = Math.max(control, -1) ;
        control = Math.min(control, 1) ;
        
        return control ;
    }
    
    // Adjust the distance from vision for the standoff
    private double adjustDistance(double visionDist, double visionBearingCWrad) 
    {    
        // get the absolute orientation from the bearing directed by PID
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
