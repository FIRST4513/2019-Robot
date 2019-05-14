package robot.utils;

// This interface is used to transfer the SuperPidIO's calculated PIDoutput value, which received
// a value from the PID Controller and then possibly modified it further. This new value is then
// sent to the SubSystem through this interface to drive the motor.

public interface SuperPidOutput{

	public abstract void superPIDWrite(double out);
	
}
