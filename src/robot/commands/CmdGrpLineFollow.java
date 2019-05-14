package robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.Robot;

public class CmdGrpLineFollow extends CommandGroup
{
    public CmdGrpLineFollow(double baseAccel,    // base drive power
                String path,         // filename for image of path
                double pix_per_in,   // image scale in pix/in
                double linewinches,  // width of the line in inches
                double startTheta,   // start orient deg CCW relative to field
                double endTheta) {   // desired ending theta
                                     // Pass Double.NaN if don't care
        
        addSequential(new CmdLineFollow(baseAccel, path, pix_per_in, 
                    linewinches, startTheta, endTheta));
        
        if (!Double.isNaN(endTheta)) {
            addSequential(new CmdRotation(startTheta, endTheta, false));
        }
    }
    
    @Override 
    public void end() {
    	System.out.println("CmdGrpLineFollow ended, writing log");
    	Robot.logger.appendLog("CmdGrpLineFollow.log");
    }
}
