package robot.utils;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import robot.Robot;

public abstract class SuperPid implements PIDSource, PIDOutput 
{
    public enum PidStatus {RUNNING, ATEND ,STOPPED} ; 
	
    // member variables
    private double mPidTarget = 0;          // distance, angle, posn, ...
    private double mRateLimitTarget = 0;    // setpoint-limited target
    private double mCurrentVal = 0;
    private boolean mIsActive = false;      // are we handling PID outputs
    //private int mStatus = 0;
    //private int mLogCounter = 0;            // used to filter the volume
    public PidOptions mOptions = null ;
    private double mInput, mRawOutput, mPidOut, mSetpoint ;

    // the actual PID controller
    private PIDController mPIDController = null ;
    double  mKp, mKi, mKd, mKf;
    
    // where's the hold value you ask?
    // it's not here yet, because I'm wondering if we can't just let the PID
    // keep running, making small adjustments if we fall off the hold position.
    // That approach implies either an integral term or a FF term (and the 
    // FF term might be pretty close to what Dennis is thinking of as the hold
    // value, the other terms then just making small adjustments to that if
    // we are near the target).
    // --- Constructors ---
    // the target only matters here if using setpoint ramping
    // it will matter to a derived class, which is responsible for setting it
    public SuperPid(double target, double Kp, double Ki, double Kd, double Kf) {
        this(target, Kp, Ki, Kd, Kf, null);
    }

    public SuperPid(double target, double Kp, double Ki, double Kd, double Kf, PidOptions options) {
        mPidTarget = target ;
        mRateLimitTarget = 0 ;
        mKp = Kp ;
        mKi = Ki ;
        mKd = Kd ;
        mKf = Kf ;
        // use default options
        if (options != null) 
           mOptions = options ;
        else {
           mOptions = new PidOptions() ;
        }
        mInput = 0;
        mRawOutput = 0;
        mPidOut = 0;
        mSetpoint = 0;
        
        // create a PID controller (or could do this in start)
        mPIDController = new PIDController(Kp, Ki, Kd, Kf, this, this) ;
    }

    public void setOptions(PidOptions opt) {
        if (opt != null) { 
            mOptions = opt ;
        }
    }

    // these must be implemented by a subclass that knows what, specifically,
    // this PidIO is for - which motor(s) and sensor(s) are used 
    abstract public void applyPidOutput(double output) ;
    
    abstract public double getPidInput() ;
    
    abstract public void updatePidStatus(PidStatus status, double input, double rawOutput, double pidOut, double setpoint ) ;
    
    // ************************************************************************
    // The PIDSource and PIDOutput interfaces
    // ************************************************************************
    @Override
    public void pidWrite(double pidOutput) {
        mRawOutput = pidOutput;

        // ------ if inActive just get out ------
        if (!mIsActive) {
            return;
        }
        
        //System.out.println("Raw PID Cmd=" + pidOutput);
        // --------- Log PID output every fifth update ------
        //if (mLogCounter == 0) {
        //    System.out.println("*** PIDOUT=" + pidOutput + " ***");
        //}
        //mLogCounter = (mLogCounter + 1) % 5;
        // --------- Modify output using Bang if Bang if requested ---------
        if (mOptions.bangBang == true) {
            pidOutput = driveBangBang(pidOutput);
        }
        mPidOut = pidOutput;
        // --------- Send pidOutput to wherever it needs to go ---------
        // the subclass must provide an implementation of this
        applyPidOutput(pidOutput);
        updatePidStatus(  PidStatus.RUNNING  , mInput, mRawOutput, mPidOut, mSetpoint ) ;			
        // --------- Check if we have reached the destination ---------
        
        double error = Math.abs(mCurrentVal - mPidTarget);  
        if (error <= mOptions.absTol) {
        	// We are in the tolereance zone for the end
        	if (!mOptions.keepRunning) {          	  
              	  Robot.logger.appendLog("PID has detected its time to stop");
            	  System.out.println("PID has detected its time to stop"); 
            	  stop();
            }
        	// We are in the tolereance zone for the end but we are not to stop so just send msg back
            updatePidStatus(  PidStatus.ATEND  , mInput, mRawOutput, mPidOut, mRateLimitTarget ) ;			// Stopped         	
        }

        // --------- Recalculate set point ---------
        if (mOptions.setpointRamping) {
            incrRampSetpoint();
            mPIDController.setSetpoint(mRateLimitTarget);
        }
        mSetpoint = mRateLimitTarget;
    }

    @Override
    public double pidGet() {
    	mCurrentVal = getPidInput();
        mInput = mCurrentVal;
        //System.out.println("Current val = " + mCurrentVal + " Target val=" + mRateLimitTarget) ;
        return mCurrentVal ;
    }

    // ------------------------------------------------------------
    // --- Start or stop the responses to the PID
    // ------------------------------------------------------------
    public void start() {
        // set up the PID
        mPIDController.setOutputRange(mOptions.outputMin, mOptions.outputMax);
        if (mOptions.inputwrap) {
            mPIDController.setInputRange(mOptions.inputMin, mOptions.inputMax);
            mPIDController.setContinuous(true);    
        }

        // set the initial setpoint
        if (mOptions.setpointRamping) {
            mRateLimitTarget = mOptions.rampRate ;
        } else {
            mRateLimitTarget = mPidTarget ;
        }
        mSetpoint = mRateLimitTarget;
        
        // enable PID output processing
        mIsActive = true;
        updatePidStatus( PidStatus.RUNNING   , mInput, mRawOutput, mPidOut, mSetpoint) ;
        
        // start the PID
        mPIDController.setSetpoint(mRateLimitTarget);
        mPIDController.enable();
    }

    public void stop() {
        mIsActive = false;
        updatePidStatus( PidStatus.STOPPED  , mInput, mRawOutput, mPidOut, mSetpoint ) ;			// Stopped
        mPIDController.disable();
    }

    /**
     ***********************************
     * local methods
     *********************************
     */   
    // increment the set point return true if done incrementing
    private boolean incrRampSetpoint() {
        if (mPidTarget > 0) {
            if (mPidTarget > mRateLimitTarget) {
                mRateLimitTarget += mOptions.rampRate ;
                // don't overshoot the eventual target though
                if (mRateLimitTarget > mPidTarget) {
                    mRateLimitTarget = mPidTarget ;
                }
                return false;
            }
            return true ;
        } else if (mPidTarget < mRateLimitTarget) {
                mRateLimitTarget -= mOptions.rampRate ;
                // don't overshoot the eventual target though
                if (mRateLimitTarget < mPidTarget) { 
                    mRateLimitTarget = mPidTarget ;
                }
                return false;
            }
        	return true;
    }

    // apply bang-bang drive
    private double driveBangBang(double cmd) {
        if ( (cmd < mOptions.bangBangUpperThresh) && (cmd > mOptions.bangBangLowerThresh)) {
        	//System.out.println("Lower BB Detected lwr=" + mOptions.bangBangUpperThresh);
            return mOptions.bangBangUpperThresh;
        }
        if ((cmd > -mOptions.bangBangUpperThresh) && (cmd < -mOptions.bangBangLowerThresh)) {
        	//System.out.println("Lower BB Detected lwr=" + -mOptions.bangBangUpperThresh);
            return -mOptions.bangBangUpperThresh;
        }
        return cmd;
    }

    // ----------------------------------------------------------
    //      Other PIDSource Interface Methods
    // ----------------------------------------------------------
    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        // nothing to do
    }

    @Override
    public PIDSourceType getPIDSourceType() {

        return PIDSourceType.kDisplacement;
    }
}
