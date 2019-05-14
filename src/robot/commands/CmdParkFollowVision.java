package robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.utils.ParkFdbkController;

// NOTE:
// This command could be combined with CmdParkToXY in the following way:
// Start with input from the vision system as done here
// If the message time stamps indicate that it has been too long since the
// last update, simply calculate the bearing and distance for the next
// update as is done in CmdParkToXY
// When the vision system comes back online, switch back

public final class CmdParkFollowVision extends Command 
{    
    // for computing deltaT for the controller
    // (this could also be done in the controller update method)
    private double mPreviousTime = -1;
    
    // the controller
    ParkFdbkController mController ;
    
    double mStartOrientationDegCCW ;
    double mFinalOrientationDegCCW ;

    // constructor
    // Standoff is the absolute (playfield) distances to stand off
    // from the target. This is passed onto the controllers so that 
    // they can adjust distance and bearing from the Vision system.
    // Pass Double.NaN if don't care about final orientation
    public CmdParkFollowVision(double standoffXft, double standoffYft,
            double initialOrientDegCCW, double finalOrientDegCCW) {

        requires(Robot.drivetrain);
        requires(Robot.visionSubSys) ;
        
        // create the controller
        mController = new ParkFdbkController(standoffXft, standoffYft, 
                initialOrientDegCCW, finalOrientDegCCW);
       
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
        double dist = Robot.visionSubSys.getDistFt() ;
        double bearing = Robot.visionSubSys.getBearingDegCW();
        mController.start(dist,bearing);
        
        Robot.logger.appendLog("CmdParkFollowVision Init");
    	Robot.drivetrain.setLoggingOn();
    }

    // This is called every 50 msec while this command is active
    @Override
    protected void execute() {
        // get deltaT in seconds for the controller
        double now = System.currentTimeMillis();
        double elapsed = (now - mPreviousTime) / 1000.0 ;        
        mPreviousTime = now;  
        
        double dist = Robot.visionSubSys.getDistFt() ;
        double bearing = Robot.visionSubSys.getBearingDegCW() ;
        double orient = Robot.drivetrain.getOrientDegCCW() ;        

        // get inputs to the two PIDs
        // TODO
        // if the Vision updates drop, revert to what is done in CmdParkToXY
        // this approach would also apply to CmdDualPidFollowVision
        /****
        if (Double.isNaN(dist) {
             // get dist and bearing like we do in CmdParkToXY 
        }
        ****/
        
        // get control output from the two PIDs
        // in the actual robot we'll use elapsed here
        ParkFdbkController.MotorControlStruct 
                ctrl = mController.update(dist, bearing, orient, elapsed) ;

        // command the drivetrain
        Robot.drivetrain.tankDrive(ctrl.left, ctrl.right);
        // NOTE:
        // I have tested this approach in simulation, but ONLY for an arcadeDrive
        // that does NOT square input for decreased sensitivity at low values
        // That is the default for wpilib, but can be controlled with a 3rd argument
        // Robot.drivetrain.arcadeDrive(ctrl.left, ctrl.right);
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
        
        Robot.logger.appendLog("CmdParkFollowVision End");
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
        return "CmdParkFollowVision" ;
    }
}
