package robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.Robot;

public class CmdGrpParkToXY extends CommandGroup
{
    public CmdGrpParkToXY(double targXft, double targYft,
            double standoffXft, double standoffYft,
            double initOrientDegCCW, double finalOrientDegCCW) {
        
        addSequential(new CmdParkToXY(targXft, targYft,
                standoffXft, standoffYft, 
                   initOrientDegCCW, finalOrientDegCCW));
       
        // just in case we're off a tad
        if (!Double.isNaN(finalOrientDegCCW)) {
            addSequential(new CmdRotation(initOrientDegCCW, finalOrientDegCCW, false));
        }
    }
    
    @Override 
    public void end() {
    	System.out.println("CmdGrpParkToXY ending");
    	Robot.logger.appendLog("CmdGrpParkToXY ending");
    	Robot.logger.appendLog("CmdGrpParkToXY.csv");
    }
}
