package robot.utils;

import edu.wpi.first.wpilibj.PIDController;
import robot.subsystems.ElevSubSys.PIDMode;


public class SuperPIDController implements SuperPidStatus {
	SuperPidIO mPidIO ;
	PIDController mPidCtrl ;
 	public enum PIDMode {NULL, TOP, BOTTOM, SWITCH, SCALE, START, MOVING, HOLDING, RUNNING,  STOPPED};
 	public PIDMode mPIDMode = PIDMode.START ;  	// this keeps track of our current drive mode
	
	boolean mIsActive = false;
	double TGT_POS = 0;

	// --- Constructor ---
	public SuperPIDController(double kP, double kI, double kD, double kF, double setPoint ) {
		TGT_POS = setPoint;
		mPidIO = new SuperPidIO(TGT_POS);
		mPidCtrl = new PIDController(kP, kI, kD, kF, mPidIO, mPidIO);
		
	}
	
	// ------------------------------------------------------------
	// --- Methods to set and clear various options for PID control
	// ------------------------------------------------------------
	public void setOutputRange( double lwr, double upper) 	{ mPidCtrl.setOutputRange( lwr, upper) ; }
	public void setContinuous( boolean val) 				{ mPidCtrl.setContinuous( val) ; }		
	public void setAbsoluteTolerance( double ABS_TOL_IN) 	{ mPidCtrl.setAbsoluteTolerance( ABS_TOL_IN);}
	
    //---- A Command can call this to see if we're done  ------
    public PIDMode getPIDMode() 								{ return mPIDMode ; } // THIS NEEDS UPDATEING !!!!!!   
   
    
    // ---- A Command can call this to force a stop (e.g., from a watchdog timer) ---    
	public void stopPIDCtr() {
		// stop the drive motors and shutdown automation
		//System.out.println("****** STOPPING PID Controll *****") ;
		if (mPidIO!=null)
			mPidIO.stop();
		if (mPidCtrl!=null) {
			mPidCtrl.reset() ;
			mPidCtrl.disable();
			mPidCtrl.free();
			mPidCtrl = null ;			
		}
		mPIDMode = mPIDMode.STOPPED ;			// Need to work on status field
	}
	
	// --------------------------------------------------------------
	// --- Methods to set and clear various options for SuperPidIO
	// --------------------------------------------------------------
	// call this to Start/Stop the PID IO
	public void start() {
		mPidIO.start();
		mPidCtrl.enable();
	}
	public void stop() {
		mPidIO.stop();
		mPidCtrl.disable();
	}

	// Enable/Disable SetPoint Ramping
	public void setSetpointRamping(double rate) 	{ mPidIO.setSetpointRamping (rate) ; }
	public void clearSetpointRamping()				{ mPidIO.clearSetpointRamping () ;	}

	// Enable/Disable BangBang output modification
	public void setBBang(double lwr, double upper)	{mPidIO.setBBang( lwr, upper) ; }
	public void clearBBang() 						{mPidIO.clearBBang() ; }	
	
 	// Enable/Disable output Limiting
	public void setOutputLimiting(double limit) 	{ mPidIO.setOutputLimiting( limit ); }
	public void clearOutputLimiting() 				{ mPidIO.clearOutputLimiting(); }


	// ------------ These methods are used to get data back from SuperPidIO Routine as it respondes
	// to the PIdController
	@Override
	public void updateSetpoint(double setPt) {
		// This is returned from the SuperPidIO to do setpoint ramping
		mPidCtrl.setSetpoint(setPt);
		
	}

	@Override
	public void updateStatus(double status, double pidOut) {
		// TODO Auto-generated method stub
		
		
	}

}
