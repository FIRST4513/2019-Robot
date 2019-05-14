package robot.utils;

// This interface is used to transfer the Subsystems current position data to the SuperPidIO.
// This value may then be further modified as requested before then being sent to the PID Controller
// trough the PIDSource interface. The PidConroller will then calculate a new PIDout value. The current
// position data is also used by the SuperPIDIO to determine if we are at the end of travel.

public interface SuperPidSource {


	public abstract double superPIDGet() ;
	
}
