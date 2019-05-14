/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.Robot;

public class RotationPid 
{
    private double KP_ROT = 0.04 ;
    private double KD_ROT = .006 ; 
    private double ROT_BB_UPPER = 0 ; // 0.5 ;
    private double ROT_BB_LOWER = 0 ; // 0.001 ;
 
    private double mDesiredThetaDeg ;
    private double mLastError = 0 ;
    private boolean mIsFinished = true ;
    
    // pass final desired angle (Double.Nan if don't care)
    public RotationPid(double endTheta) {       
        KP_ROT = SmartDashboard.getNumber("BEARING_PID_KP", KP_ROT) ;
        KD_ROT = SmartDashboard.getNumber("BEARING_PID_KD", KD_ROT) ;
        ROT_BB_UPPER = SmartDashboard.getNumber("BEARING_PID_BB_UPPER", ROT_BB_UPPER) ;
        ROT_BB_LOWER = SmartDashboard.getNumber("BEARING_PID_BB_LOWER", ROT_BB_LOWER) ;
        mDesiredThetaDeg = endTheta ;    
    }

    public void start() {
        mIsFinished = false ;
        mLastError = Robot.drivetrain.getOrientDegCCW()
                     - mDesiredThetaDeg ;
    }
    
    public void stop() {
        mIsFinished = true ;
    }
    
    public boolean isFinished() {
        return mIsFinished ;
    }
    
    public double update(double orientAbsDegCCW, double dT) {
        if (mIsFinished)
            return 0.0 ;
                
        double error =  orientAbsDegCCW - mDesiredThetaDeg ;
        
        // update acceleration controls based on position error
        double deriv = (error-mLastError)/dT ;
        mLastError = error ;
        
        /****
        // stop when error starts to increase 
        // this would stop as soon as overshoot occurs
        // which wouldn't give us a chance to fix it
        // which could be bad if it is coasting
        if (error>0 && deriv>0)
            mIsFinished = true ;
        else if (error<0 && deriv<0)
            mIsFinished = true ;
        else if (error==0)
            mIsFinished = true ;
        ****/
        
        // TODO: This might be a better stopping criterion
        /****
        // stop when error and velocity are both small
        // note that these are degrees and degrees/sec
        if (Math.abs(error)<2 && Math.abs(deriv)<1)
        * mIsFinished = true ;
        ***/
        
        // simplistic stop doesn't deal with overshoot
        // KD can be tuned to prevent that, for at least some turn ranges
        // if not we need a more complex stopping criterion like the preceding
        if (Math.abs(error)<1.0)
            mIsFinished = true;

        double rotCmd = KP_ROT*error + KD_ROT*deriv ;

        // rail the accelerations to +/- 1
        rotCmd = Math.max(rotCmd, -1) ;
        rotCmd = Math.min(rotCmd, 1) ;
        
        // Bang Bang 
        if (rotCmd>ROT_BB_LOWER)
            rotCmd = Math.max(ROT_BB_UPPER, rotCmd) ;
        else if (rotCmd<-ROT_BB_LOWER) 
            rotCmd = Math.min(-ROT_BB_UPPER, rotCmd) ;
        
        return rotCmd ;
    }   
}
