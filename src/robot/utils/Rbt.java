package robot.utils;


public class Rbt {
	public final double tmtr = 22;
	public final double ROBOTFRONTTOCTR = 14.25;
	public final double WHEELBASE = 22.0;		// Distance between wheels
	
	//Robot position data
	public double	currXpos;
	public double	currYpos;
	public double 	currAngle;
	public double	leftEncDist;
	public double	rightEncDist;	
	public double	lastXpos;
	public double	lastYpos;
	public double	lastAngle;
	public double	turnRadius;

/*
	// These variables were used by vision calculation routines
	public Point 	COG;
	public double 	yaw;
	public double 	angle;
	public Point 	pos;
	public Point 	startPt;
	public Point 	endPt;
	public Point 	xAxisIntPt;
	public double 	distx, disty;
	public Segment 	snglArc;
	public Segment 	dblArc1;
	public Segment 	dblArc2;
	public Point 	ctrPt1;
	public Point 	ctrPt2;
	public double 	radius;
	public int 		testFlag;				// Code value of Dbl curve quadCalc solution
	public int		solutionFlag;
	public int 		arcDir1;				// Arc1 direction of Dbl solution
	public int 		arcDir2;
	public int 		ctr2sign;				// Used in quad calculation - Robot Position above X axis
	public int 		ctr1xsign;
	public int 		ctr1ysign;
	public Route 	rbtRoute;				// Store route segments determined by single or double solution
*/

	// ***************************************************
	//	              Rbt Class Methods
	// ***************************************************
	public Rbt(){
		// --- Rbt Class constructor ----
		currXpos = 0 ;
		currYpos = 0 ;
		currAngle = 0;
		leftEncDist = 0 ;
		rightEncDist = 0 ;
		lastXpos = 0;
		lastYpos = 0;
		lastAngle = 0;
		turnRadius = 0 ;
		
		
		/*
		COG = new Point(0,0);
		hdg = 0;
		yaw = 0;
		pos = new Point(0,0);
		startPt = new Point(0,0);
		endPt = new Point(0,0);
		xAxisIntPt = new Point(0,0);
		distx= 0;
		disty= 0;
		angle = 0;
		ctrPt1 = new Point(0,0);
		ctrPt2 = new Point(0,0);
		radius = 0;
		testFlag = 0;
		solutionFlag = 0;
		arcDir1 = 0;
		arcDir2 = 0;
		ctr2sign = 0;
		ctr1xsign = 0;
		ctr1ysign = 0;
		rbtRoute = new Route();
		snglArc = new Segment();
		dblArc1 = new Segment();
		dblArc2 = new Segment();
		*/
	}

	public Rbt(double x, double y, double angle){
		// --- Rbt Class constructor ----= 
		currXpos = x ;
		currYpos = y ;
		currAngle = angle;
		leftEncDist = 0 ;
		rightEncDist = 0 ;
		lastXpos = 0 ;
		lastYpos = 0 ;
		lastAngle = 0 ;
		turnRadius = 0 ;
	}
	
	public void setEncDist( double left, double right) {
		leftEncDist = left ;
		rightEncDist = right ;		
	}
	
	public void updateRbtPos(double left, double right) {
		// calculate new position given change in left and right wheel travel
		// may need to average last several reading to act as low pass filter

		double rightMinusLeft = right - left ;
    	double rightPlusLeft = right + left ;
    	if (rightMinusLeft == 0) rightMinusLeft = 0.00000000001 ; 	// prevent divide by zero
    	double angle =  rightMinusLeft / WHEELBASE ;		// robot current yaw heading
    	double radius =  (WHEELBASE/2) * (rightPlusLeft/rightMinusLeft) ;
    	
    	//  ---Calculate Robot position on field---
    	
    	double wheelRadius = WHEELBASE/2 ;
    	double sinAngle = Rmath.mSin(angle) ;
    	double sinDist = Rmath.mSin(rightMinusLeft/WHEELBASE) + angle ;
    	double cosAngle = Rmath.mCos(angle) ;
    	double cosDist = Rmath.mCos(rightMinusLeft/WHEELBASE) + angle ;    	
    	lastXpos = currXpos ;
    	lastYpos = currYpos ;
    	currXpos = lastXpos + (wheelRadius) * (rightPlusLeft/rightMinusLeft) * (sinDist) - sinAngle ;
    	currYpos = lastYpos + (wheelRadius) * (rightPlusLeft/rightMinusLeft) * (cosDist) - cosAngle ;    	
    	lastAngle = currAngle ;
    	currAngle = angle ;
    	turnRadius = radius ;
	}
}