/*
 * author: Paul Schimpf
 * date: 26 Jan, 2018
 */
package robot.utils;

public class PidOptions {
    public double  outputMax = 1.0 ;
    public double  outputMin = -1.0 ;
    
    public boolean setpointRamping = false;
    public double  rampRate = 0.15 ;                 // per update
    
    public boolean bangBang = false ;
    public double  bangBangUpperThresh = 0.4;
    public double  bangBangLowerThresh = 0.01;
    
    public boolean keepRunning = false ;  // should we stop ourself?
    public double  absTol = 1.0;           // stopping tolerance 
    
    public boolean inputwrap = false ;
    public double  inputMin ;
    public double  inputMax ;
    
    public PidOptions(double outmax, double outmin, 
            boolean ramp, double rate, 
            boolean bang, double upper, double lower,
            boolean keeprunning, double tol,
            boolean wrap, double inmin, double inmax) {
        outputMax = outmax ;
        outputMin = outmin ;
        setpointRamping = ramp ;
        rampRate = rate ;
        bangBang = bang ;
        bangBangUpperThresh = upper ;
        bangBangLowerThresh = lower ;
        keepRunning = keeprunning ;
        absTol = tol ;
        inputwrap = wrap ;
        inputMin = inmin ;
        inputMax = inmax ;
    }
    
    public PidOptions() {
        // take defaults in the var declarations above
    }
    
    public void setOutputRange(double outmax, double outmin) {
        outputMax = outmax ;
        outputMin = outmin ;        
    }
    
    public void setRamping(boolean ramp, double rate) {
        setpointRamping = ramp ;
        rampRate = rate ;        
    }
    
    public void setBangBang(boolean bang, double upper, double lower) {
        bangBang = bang ;
        bangBangUpperThresh = upper ;
        bangBangLowerThresh = lower ;        
    }
    
    public void setKeepRunningCriteria(boolean keeprunning, double tol) {
        keepRunning = keeprunning ;
        absTol = tol ;        
    }
    
    public void setInputWrap(boolean rangewrap, double min, double max) {
        inputwrap = rangewrap ;
        inputMin = min ;
        inputMax = max ;
    }
    
    public void setTolerance(double tol) {
    	absTol = tol ;
    }

}