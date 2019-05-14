package robot.utils;

// This interface is to provide information to and from SuperPidIO process to the SuperPidController.

public interface SuperPidStatus {

		// This method is used to pass back the newly calculated setpoint value
		// if "Setpoint Ramping" has been requested of the Pid process.
		
		public abstract void updateSetpoint(double setPoint);
		
		
		// This method provides status information from the Pid process, such as
		// running status, current pidout.
		
		public abstract void updateStatus(double status, double pidOut);
		
}
