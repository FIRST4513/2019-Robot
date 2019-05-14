package robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.Robot;

public class CmdGrpDualPidFollowVision extends CommandGroup
{
    public CmdGrpDualPidFollowVision(double standoffXft, double standoffYft,
                   double initOrientDegCCW, double finalOrientDegCCW) {
        
        addSequential(new CmdDualPidFollowVision(standoffXft, standoffYft, 
                   initOrientDegCCW, finalOrientDegCCW));
        
        // if we care about final orientation, run the rotation PID
        if (!Double.isNaN(finalOrientDegCCW)) {
            addSequential(new CmdRotation(initOrientDegCCW, finalOrientDegCCW, false));
        }

    }
    
    @Override 
    public void end() {
    	// TODO: this was for testing individual algorithms
    	// probably want to get rid of it for the production system
    	// there are similar calls in the other Command Groups
    	Robot.logger.appendLog("CmdGrpDualPidFollowVision.log");
    }
}
