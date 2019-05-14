package robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.Robot;

public class CmdGrpParkFollowVision extends CommandGroup
{
    public CmdGrpParkFollowVision(double standoffXft, double standoffYft,
                   double initOrientDegCCW, double finalOrientDegCCW) {
        
        addSequential(new CmdParkFollowVision(standoffXft, standoffYft, 
                   initOrientDegCCW, finalOrientDegCCW));
       
        // just in case we're off a tad
        if (!Double.isNaN(finalOrientDegCCW)) {
            addSequential(new CmdRotation(initOrientDegCCW, finalOrientDegCCW, false));
        }
    }    
    
    @Override 
    public void end() {
    	Robot.logger.appendLog("CmdGrpParkFollowVision.log");
    }
}
