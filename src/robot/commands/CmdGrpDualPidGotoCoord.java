package robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.Robot;

public class CmdGrpDualPidGotoCoord extends CommandGroup
{
    public CmdGrpDualPidGotoCoord(double targXft, double targYft,
            double standoffXft, double standoffYft,
            double initOrientDegCCW, double finalOrientDegCCW) {
        
        addSequential(new CmdDualPidToCoord(targXft, targYft,
                				standoffXft, standoffYft, 
                				initOrientDegCCW, finalOrientDegCCW));
        
        // if we care about final orientation, run the rotation PID
        if (!Double.isNaN(finalOrientDegCCW)) {
            addSequential(new CmdRotation(initOrientDegCCW, finalOrientDegCCW, false));
        }
    }
    
    @Override 
    public void end() {
        Robot.logger.appendLog("CmdGrpDualPidGotoCoord.log");
        Robot.logger.printLog();
    }
}
