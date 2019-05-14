package robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.utils.RotationPid;

/* This class corresponds to a robot command class
 */
public final class CmdRotation extends Command 
{
	// TODO: might want to load this from shuffleboard
	// This ignores the call to this command if we're close enough
	// Why not just NOT make the call?
	// Because it wasn't clear to me that in a Command Group
	// you could wait for the output of one command before doing
	// an addSequential()
	private final double ORIENT_THRESH = 5 ;    // degrees
	private boolean mIgnoreThis = false ;
	 
    // for computing deltaT for the controller
    // (this could also be done in the controller update method)
    private double mPreviousTime = -1;
    
    // the controller
    RotationPid mRotationController ;
    
    double mStartOrientationDegCCW ;
    boolean mResetYawAndLoc ;

    // constructor
    public CmdRotation(double initialOrientDegCCW, double finalOrientDegCCW, boolean resetYawAndLoc) {
        requires(Robot.drivetrain);

        // ?????????????????????????????????????????????????????????????///
        
        //requires(Robot.robotVision) ;
        
        mRotationController = new RotationPid(finalOrientDegCCW) ;        
        mStartOrientationDegCCW = initialOrientDegCCW ;
        mResetYawAndLoc = resetYawAndLoc ;
    }
    
    @Override
    protected void initialize() {
    	
        // TODO: 
        // See comments above about the member variable mIgnore
        // I have NOT tested it even in simulation so it is commented 
        // out here. The approach would hold in the other feedback control Commands as well
        /****
        if (Math.abs(Robot.drivetrain.getOrientDegCCW()-finalOrientDegCCW) < ORIENT_THRESH) {
        	mIgnoreThis = true ;
        	return ;
        }
        ****/

        // initialize for computing deltaT
        mPreviousTime = System.currentTimeMillis(); 
        // the controllers don't use position data
        // but they do need Yaw Data, which can come from
        // either the IMU or the Position Tracker. We
        // initialize both so we can change our minds
        // in the drivetrain code later on, depending
        // upon the relative performance of the two approaches.
        if (mResetYawAndLoc) {
        	Robot.drivetrain.resetEncodersAndStats();    
           	Robot.drivetrain.resetPosition(true);        	
        }
    	Robot.drivetrain.setInitialOrientationDegCCW(mStartOrientationDegCCW);        
        mRotationController.start();    
        
        Robot.logger.appendLog("CmdRotation Init");
    	Robot.drivetrain.setLoggingOn();
    }

    // This is called every 50 msec while this command is active
    @Override
    protected void execute() {
    	// if (mIgnoreThis) return ;

    	// get deltaT in seconds for the controller
        double now = System.currentTimeMillis();
        double elapsed = (now - mPreviousTime) / 1000.0 ;        
        mPreviousTime = now;   
        
        // UPDATE
        double orientAbsDegCCW = Robot.drivetrain.getOrientDegCCW() ;
        double cmd = mRotationController.update(orientAbsDegCCW, elapsed);
        Robot.drivetrain.tankDrive(cmd, -cmd);                   
    }
    
    @Override
    protected boolean isFinished() {
    	if (mIgnoreThis) return true ;
    	return (mRotationController.isFinished()) ;
    }
    
    // Called once after isFinished returns true
    @Override
    protected void end() {
    	// if (mIgnoreThis) return ;
        mRotationController.stop();
        Robot.drivetrain.tankDrive(0, 0);
        Robot.logger.appendLog("CmdRotation End");
    	Robot.drivetrain.setLoggingOff();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    	end();
    }

    @Override public String toString() {
        return "CmdRotation" ;
    }
}
