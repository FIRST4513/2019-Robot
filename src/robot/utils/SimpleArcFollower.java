package robot.utils;

/***
 * This class computes how far the robot is currently from an arc defined by a portion of a circle, 
 * and recommends left and right drive power settings to get back to that circle. Corrections are
 * expected to be incremental, so it needs to be started on the arc, and it needs to be asked for
 * updates on a timely basis as the robot is moving. Kp will need to be tuned. Suggestions for how
 * to get an initial guess: suppose distances are in inches. If one wanted a 10% increase in left or
 * right power, relative to the base setting, for an error of 1 inch off the arc, then set Kp=0.1.
 * At present this is written to only work with forward travel. Making it work with backwards 
 * travel won't be difficult. At present there are not any safeguards against misuse, such as 
 * passing a 0 in the end yaw, or an arc radius of 0 or less, calling for an update when the
 * robot is far away from the path, etc. 
 * 
 * Example use in a RobotBuilder Command:
 * 1) In initialize():
 * Create an object of this class, giving it the required information, and initializing a timeout.
 * The required information includes the geometry of the arc being followed. The assumption is that 
 * the robot is currently placed at one end of that arc, facing along a tangent to the arc. 
 * The command's initialize() method should also reset the current position and yaw to zero, however 
 * that is being tracked. 
 * 
 * 2) In execute():
 * If the current Yaw has reached the end yaw, then set a flag for the isFinished() method. Otherwise,
 * ask the FollowArcController for an update, passing it the current position and yaw, relative to the 
 * starting position. Note that it returns a null if the current Yaw has reached or gone past the end yaw, 
 * so that could be used as a convenient test as to whether we have reached the end. If it returns an 
 * object of type SimpleArcFollower.FollowArcOutput (you'll need to import that), command the left and 
 * right motors as that object suggests (the member vars for left and right are public).
 * 
 * 3) In isFinished():
 * If the flag referred to above is true, or the timeout has expired, return true. Otherwise 
 * return false. 
 * 
 * @author Paul Schimpf
 *
 */
public class SimpleArcFollower 
{
	public static class FollowArcOutput {
		public double leftPwr = 0 ;
		public double rightPwr = 0 ;
	}
	
	private double mArcRad ;
	private double mEndYaw ;
	private double mArcCenterX ;
	private double mArcCenterY ;
	private double mBasePwr ;
	private double mKp ;
	
	/***
	 * Constructor. Y is the direction the robot is facing, X is to the right. Distance Units are whatever 
	 * you like. Angle units are degrees.
	 * 
	 * @param arcRad    The radius of the arc, in whatever units you choose. Positive only please.
	 * @param centerX   The x coord of the arc center, relative to the start.
	 * @param centerY   The y coord of the arc center, relative to the start.
	 * @param endYawDeg The ending yaw of the arc (the starting yaw is 0 by definition). A positive
	 *                  endYaw indicates a CW turn, and a negative endYaw indicates a CCW turn. That
	 *                  also means that it must be in the range -180 to 180, so you cannot go more
	 *                  than a half circle. Also, an endYaw of 0 wouldn't make any sense.
	 * @param basePower The base power desired, from 0 to 1. The suggested left and right settings
	 *                  will keep this as the average between sides, as long as that doesn't mean
	 *                  going beyond 1 on a side.
	 * @param Kp        The gain factor. See class description above for a starting guess suggestion.
	 */
	public SimpleArcFollower(double arcRad, double centerX, double centerY, double endYaw, double basePower, double Kp) {
		mArcRad = arcRad ;
		mArcCenterX = centerX ;
		mArcCenterY = centerY ;
		mEndYaw = endYaw ;
		mBasePwr = basePower ;
		mKp = Kp ;
	}
	
	public FollowArcOutput getUpdate(double x, double y, double yaw) {
		// DS - I Wanted to let calculations continue and have the Command determine stopping
		//if (mEndYaw>=0 && yaw>=mEndYaw) 
		//	return null ;
		//if (mEndYaw<=0 && yaw<=mEndYaw)
		//	return null ;
		
		// create an output object
		FollowArcOutput out = new FollowArcOutput() ;
		
		// get our distance from the arc (pos is outside, negative is inside)
		double distToCenter = Math.sqrt((x-mArcCenterX)*(x-mArcCenterX)+(y-mArcCenterY)*(y-mArcCenterY)) ;
		double outsideDist = distToCenter - mArcRad ;
		
		// adjust the left and right power
		if (mEndYaw>0) { // CW turn
			out.leftPwr = mBasePwr * (1 + mKp*outsideDist) ;
			out.rightPwr = mBasePwr * (1 - mKp*outsideDist) ;
		}
		else {
			out.leftPwr = mBasePwr * (1 + mKp*outsideDist) ;
			out.rightPwr = mBasePwr * (1 - mKp*outsideDist) ;			
		}
		
		double max = Math.max(out.leftPwr, out.rightPwr) ;
		if (max>1) {
			out.leftPwr = out.leftPwr / max ;
			out.rightPwr = out.rightPwr / max ;
		}
		
		return out ;
	}
}
