package robot.commands;

import robot.Robot;
import robot.utils.*;
import edu.wpi.first.wpilibj.command.Command;

/* This class corresponds to a robot command class
 */
public final class CmdDualPidToCoord extends Command 
{    
    // for computing deltaT for the controller
    // (this could also be done in the controller update method)
    private double mPreviousTime = -1;
    
    // the dual controllers
    DistancePid mDistController ;
    BearingPid  mBearingController ;
    
    double mStartOrientationDegCCW ;
    double mFinalOrientationDegCCW ;
    double mTargetX ;
    double mTargetY ;

    // constructor
    // Standoff is the absolute (playfield) distances to stand off
    // from the target. This is passed onto the controllers so that 
    // they can adjust distance and bearing from the Vision system.
    // Pass Double.NaN if don't care about final orientation
    public CmdDualPidToCoord(double targXft, double targYft,
            double standoffXft, double standoffYft,
            double initialOrientDegCCW, double finalOrientDegCCW) {

        requires(Robot.drivetrain);
        
        // create the two controllers
        mDistController = new DistancePid(standoffXft, standoffYft);
        mBearingController = new BearingPid(standoffXft, standoffYft) ;
        
        mTargetY = targYft ;
        mTargetX = targXft ;
        mStartOrientationDegCCW = initialOrientDegCCW ;
        mFinalOrientationDegCCW = finalOrientDegCCW ;
    }
    
    @Override
    protected void initialize() {
        setTimeout(0.5);
        // initialize for computing deltaT
        mPreviousTime = System.currentTimeMillis(); 
        // the controllers don't use position data
        // but they do need Yaw Data, which can come from
        // either the IMU or the Position Tracker. We
        // initialize both so we can change our minds
        // in the drivetrain code later on, depending
        // upon the relative performance of the two approaches.
        Robot.drivetrain.resetGyro();   
    	Robot.drivetrain.resetEncodersAndStats();    
        Robot.drivetrain.resetPosition(true);
    	Robot.drivetrain.setInitialOrientationDegCCW(mStartOrientationDegCCW);  
        double dist = getDistToTargFt() ;
        double bearing = getBearingToTargDegCW() ;
        double orient = Robot.drivetrain.getOrientDegCCW() ;
        mDistController.start(dist, bearing);
        mBearingController.start(dist, bearing, orient);
        
        Robot.logger.appendLog("CmdDualPidToCoord Init");
    	Robot.drivetrain.setLoggingOn();
    }

    // This is called every 50 msec while this command is active
    @Override
    protected void execute() {
        // get deltaT in seconds for the controller
        double now = System.currentTimeMillis();
        double elapsed = (now - mPreviousTime) / 1000.0 ;        
        mPreviousTime = now;   
        
        // get the inputs to the two PIDs
        double dist = getDistToTargFt() ;
        double bearing = getBearingToTargDegCW() ;
        double orient = Robot.drivetrain.getOrientDegCCW() ;
        
        // get control output from the two PIDs
        // in the actual robot we'll use elapsed here
        double distControl = mDistController.update(dist, bearing, elapsed) ;
        double bearingControl = mBearingController.update(dist, bearing, orient, elapsed);
        // combine them
        double left = (distControl+bearingControl)  ;  // / 2.0 ;
        double right = (distControl-bearingControl) ; // / 2.0 ;

        System.out.println(System.currentTimeMillis()) ;
        System.out.printf("leftpwr=%g rghtpwr=%g\n", left, right);

        // rail the accelerations to +/- 1
        /***/
        left = Math.max(left, -1) ;
        left = Math.min(left, 1) ;
        right = Math.max(right, -1) ;
        right = Math.min(right, 1) ;
        /***/
        
        // command the drivetrain
        Robot.drivetrain.tankDrive(left, right);
        Robot.drivetrain.putZoneData( 11, dist*12, distControl, bearing, bearingControl );
    }
    
    @Override
    protected boolean isFinished() {
        if (isTimedOut()) {
            System.out.println("timed out") ;
    		String line = new String("CmdDualPidToCoord - Has timed out !!");
            Robot.logger.appendLog(line);
    		System.out.println(line) ;
    		return true;								// used in all modes
    	}
    	
        // we're done when distance pid is done
        boolean distFin = mDistController.isFinished() ;
        if (distFin) {
            System.out.println("distance finished");
            Robot.drivetrain.tankDrive(0, 0);
            mDistController.stop();
            mBearingController.stop();
        }
        return distFin ;
    }
    
    // Called once after isFinished returns true
    @Override
    protected void end() {
        mDistController.stop();
        mBearingController.stop();
        Robot.drivetrain.tankDrive(0, 0);
        
        Robot.logger.appendLog("CmdDualPidToCoord End");
    	Robot.drivetrain.setLoggingOff();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    	end();
    }

    @Override public String toString() {
        return "CmdDualPidToCoord" ;
    }

    private double getDistToTargFt() {
        // What we do here is compare the absolute target location
        // to our current location, and generate a bearing and distance.
        // IOW, what a vision system would be doing for a visually
        // acquired target.
        // start by getting the current location (from drivetrain posn tracker)
        double currX = Robot.drivetrain.getXinFt() ;
        double currY = Robot.drivetrain.getYinFt() ;
        System.out.printf("currx=%g curry=%g\n", currX, currY);
        double deltaX = mTargetX - currX ;
        double deltaY = mTargetY - currY ;
        System.out.printf("delx=%g dely=%g\n", deltaX, deltaY);
        return Math.sqrt(deltaX*deltaX + deltaY*deltaY) ;
    }

    private double getBearingToTargDegCW() {
        double currX = Robot.drivetrain.getXinFt() ;
        double currY = Robot.drivetrain.getYinFt() ;
        double deltaX = mTargetX - currX ;
        double deltaY = mTargetY - currY ;    
        double psiCCWdeg = Math.atan(deltaY/deltaX)*180.0/Math.PI ;
        double botAbsOrientCCW = Robot.drivetrain.getOrientDegCCW();
        return (botAbsOrientCCW - psiCCWdeg) ;
    }
}
