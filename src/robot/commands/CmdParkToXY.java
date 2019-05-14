package robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.utils.ParkFdbkController;

// NOTE:
// This command could be combined with CmdParkFollowVision in the following way:
// Start with input from the vision system as done there
// If the message time stamps indicate that it has been too long since the
// last update, simply calculate the bearing and distance for the next
// update as is done here
// When the vision system comes back online, switch back

public final class CmdParkToXY extends Command 
{    
    // for computing deltaT for the controller
    // (this could also be done in the controller update method)
    private double mPreviousTime = -1;
    
    // the controller
    ParkFdbkController mController ;
    
    double mStartOrientationDegCCW ;
    double mFinalOrientationDegCCW ;
    double mTargetX ;
    double mTargetY ;

    // constructor
    // Standoff is the absolute (playfield) distances to stand off
    // from the target. This is passed onto the controllers so that 
    // they can adjust distance and bearing from the Vision system.
    // Pass Double.NaN if don't care about final orientation
    public CmdParkToXY(double targXft, double targYft,
            double standoffXft, double standoffYft,
            double initialOrientDegCCW, double finalOrientDegCCW) {

        requires(Robot.drivetrain);


        // ????????????????????
        
        //requires(Robot.robotVision) ;
        
        // create the controller
        mController = new ParkFdbkController(standoffXft, standoffYft, 
                initialOrientDegCCW, finalOrientDegCCW);
       
        mTargetY = targYft ;
        mTargetX = targXft ;
        mStartOrientationDegCCW = initialOrientDegCCW ;
        mFinalOrientationDegCCW = finalOrientDegCCW ;
    }
    
    @Override
    protected void initialize() {
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
        mController.start(dist,bearing);
        
        Robot.logger.appendLog("CmdParkToXY Init");
    	Robot.drivetrain.setLoggingOn();
    }

    // This is called every 20 msec while this command is active
    @Override
    protected void execute() {
        // get deltaT in seconds for the controller
        double now = System.currentTimeMillis();
        double elapsed = (now - mPreviousTime) / 1000.0 ;        
        mPreviousTime = now;  
        
        // get inputs to the controller
        double dist = getDistToTargFt() ;
        double bearing = getBearingToTargDegCW() ;
        double orient = Robot.drivetrain.getOrientDegCCW() ;        
        
        // get control output from the controller
        // in the actual robot we'll use elapsed here
        ParkFdbkController.MotorControlStruct 
                ctrl = mController.update(dist, bearing, orient, elapsed) ;

        // command the drivetrain
        Robot.drivetrain.tankDrive(ctrl.left, ctrl.right);
        // Robot.drivetrain.setArcadeDrive(ctrl.left, ctrl.right);
    }
    
    @Override
    protected boolean isFinished() {
        // we're done when distance pid is done
        boolean distFin = mController.isFinished() ;
        if (distFin) {
            Robot.drivetrain.tankDrive(0, 0);
            mController.stop();
        }
        return distFin ;
    }
    
    // Called once after isFinished returns true
    @Override
    protected void end() {
        mController.stop();
        Robot.drivetrain.tankDrive(0, 0);
        
        Robot.logger.appendLog("CmdParkToXY End");
    	Robot.drivetrain.setLoggingOff();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    	end();
    }

    @Override 
    public String toString() {
        return "CmdParkToXY" ;
    }
    
    private double getDistToTargFt() {
        // What we do here is compare the absolute target location
        // to our current location, and generate a bearing and distance.
        // IOW, what a vision system would be doing for a visually
        // acquired target.
        // start by getting the current location (from drivetrain posn tracker)
        double currX = Robot.drivetrain.getXinFt() ;
        double currY = Robot.drivetrain.getYinFt() ;
        double deltaX = mTargetX - currX ;
        double deltaY = mTargetY - currY ;
        return Math.sqrt(deltaX*deltaX + deltaY*deltaY) ;
    }

    private double getBearingToTargDegCW() {
        double currX = Robot.drivetrain.getXinFt() ;
        double currY = Robot.drivetrain.getYinFt() ;
        double deltaX = mTargetX - currX ;
        double deltaY = mTargetY - currY ;    
        double psiCCWdeg = Math.atan2(deltaY,deltaX)*180.0/Math.PI ;
        double botAbsOrientCCW = Robot.drivetrain.getOrientDegCCW();
        return (botAbsOrientCCW - psiCCWdeg) ;
    }    
}
