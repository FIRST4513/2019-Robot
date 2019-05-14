package robot.utils;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class SuperPidIO implements PIDSource, PIDOutput {

	SuperPidSource mGet;
	SuperPidOutput mOut;
	SuperPidStatus mSuperPid;
	
	private double		mTgtPosIn = 0 ;           	// the total path distance
	private double      mRateLimitTargetDist = 0  ;	// the present setpoint-limited distance
    private boolean     mIsActive = false ;
    private int         mLogCounter = 0 ;           // used to filter the volume
    private boolean 	mSetpointRampingFlag = false;
 	private double 		mMove_Rate = 0.15 ;  // 3 ft/sec*.05 sec/update = 0.15 ft/update = 1.8 in/update
 	
    private boolean		mBBangFlag = false ;
	private double 		mBBANG_Upper_Thresh = 0.4 ; 
	private double 		mBBANG_Lower_Thresh = 0.01 ;
	
    private boolean		mOutputLimitFlag = false;
	private double		mOutputLimit = 0;       
 	private double 		mABS_Tol_In = 1.0 ;	// Tolerance value to determine completion
	
	private double		mPidOutput = 0; 
	double				TGT_POS;
	
	// --- Constructor ---
	public SuperPidIO(double tgt) {
		TGT_POS = tgt;
	}
	
	
	// ************************************************************************
	//    Main Methods to Handle PidController pidWrite & PidGet PID control
	//              This is where all the work is done
	// ************************************************************************
	
	@Override
	public void pidWrite(double pidOutput) {
		// ------ if inActive just get out ------
		if (!mIsActive) return ;

		// --------- Log data every fith time through loop ------
    	if (mLogCounter==0) System.out.println("*** PIDOUT=" + pidOutput + " ***") ;
		mLogCounter = (mLogCounter+1)%5 ;
    	
		// --------- Modify output using Bang if Bang if requested ---------
		if (mBBangFlag == true) {
			pidOutput = driveBangBang(pidOutput) ; 
		}
		
		// --------- Limit output to a max value if requested ---------
		if (mOutputLimitFlag == true ) {
			if (( pidOutput <= 0) && ( pidOutput < -mOutputLimit))
				pidOutput = - mOutputLimit;
			if (( pidOutput >= 0) && ( pidOutput > mOutputLimit))
				pidOutput =  mOutputLimit;
		}

		// --------- Send pidOutput to motor ---------
		mOut.superPIDWrite(pidOutput);
    	mPidOutput = pidOutput;

    	// --------- Check if we have reached the destination ---------    
    	double error = Math.abs(mGet.superPIDGet() - mTgtPosIn) ;
	    if (error <= mABS_Tol_In) {
	    	// ***** this needs work ******************************
	    	//hold();				// This sends enough power to hold position
	    }
	        	
		// --------- Recalculate set point using Setpoint Ramping, if requested ---------
	    if (mSetpointRampingFlag == true) {
	    	incrRampSetpoint() ;
			//mPidController.setSetpoint(mRateLimitTargetDist) ;
			mSuperPid.updateSetpoint(mRateLimitTargetDist);
	    }
		
	}
	
	
	// ----------------- Optional Pid Routines -------------------------------------
	private boolean incrRampSetpoint() {
		// This currently is only used in drive straight, to smooth acceleration
		if (mTgtPosIn > mRateLimitTargetDist) {
			mRateLimitTargetDist += mMove_Rate ;				// Lets creep up to final target
		    if (mRateLimitTargetDist > mTgtPosIn)
		    	mRateLimitTargetDist = mTgtPosIn ;				// We  overshot so reset to final tgt pos   
		   return false ;				
		}
		return true ;
	}
	
	private double driveBangBang(double cmd) {
		// This currently is only used in point turn, to overcome friction forces on starting turn
		if ((cmd<mBBANG_Upper_Thresh) && (cmd>mBBANG_Lower_Thresh))
			return mBBANG_Upper_Thresh ;
		if ((cmd>-mBBANG_Upper_Thresh) && (cmd<-mBBANG_Lower_Thresh))
			return -mBBANG_Upper_Thresh ;
		return cmd ;
	}
	
	
	@Override
	public double pidGet() {
		// TODO Auto-generated method stub
		double x = mGet.superPIDGet();
		return x;
	}
	
	// ------------------------------------------------------------
	// --- Methods to set and clear various options for PID control
	// ------------------------------------------------------------

	// --- call this to Start/Stop the PID IO ---
	
	public void setTgtPos(double target) {
		TGT_POS = target;
	}
	
	public void stop() {
		mIsActive = false ;
		}	

	public void start() {
		// set the first setpoint, enable the PID, enable the PID I/O 
		if (mSetpointRampingFlag == true) {
			incrRampSetpoint() ;
			mSuperPid.updateSetpoint(mRateLimitTargetDist);		// adjust the set point to this value
		} else {
			mSuperPid.updateSetpoint(TGT_POS) ; 		// Set the setpoint
		}
		mIsActive = true ;
	}

	
	public void setSetpointRamping(double rate) 	{
		// Enable SetPoint Ramping
		mSetpointRampingFlag = true;
		mMove_Rate = rate;
		}
	
	public void clearSetpointRamping()	{
		mSetpointRampingFlag = false;
		mMove_Rate = 0;			
	}
	public void setBBang(double lwr, double upper) {
		// Enable BangBang output modification
		mBBangFlag = true;
		mBBANG_Upper_Thresh = upper ; 
		mBBANG_Lower_Thresh = lwr;
		}
	public void clearBBang() {
		 mBBangFlag = false;
			mBBANG_Upper_Thresh = 0 ; 
			mBBANG_Lower_Thresh = 0;
		}
	
	public void setOutputLimiting(double limit) 	{
		// Enable output Limiting
		mOutputLimitFlag = true;
		mOutputLimit = limit;
		}
		
	public void clearOutputLimiting() 	{
		mOutputLimitFlag = true;
		mOutputLimit = 0;
		}


	// ----------------------------------------------------------
	//      Other Pid Controller Interface Implementations
	// ----------------------------------------------------------
	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub	
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return PIDSourceType.kDisplacement ;
	}
	

}
