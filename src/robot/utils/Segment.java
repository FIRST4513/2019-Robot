package robot.utils;

import java.lang.Math;

public class Segment {
	// --- Segment Data Elements ---
	public char type;				// 'N' not active, 'A' Arc,  'S' straight line
	public Point startPt;
	public Point ctrPt;
	public Point endPt;
	public double len;
	public double radius;
	public int dir;                // -1 = counter clockwise, +1 = clokwise
	public double startAngle;		// Heading of Robot at start of Arc Pt1
	public double endAngle;		// Heading of Robot at end of Arc Pt2
	public double arcAngle;		// Angle from startPt to endPt described by arc segment
	public double vRatio;			// Ratio of left to right motor speed for this radius
	public double vLeft;			// Velocity of Left Motor -1 to +1
	public double vRight;			// Velocity of Right Motor -1 to +1

	public static final int CW = +1;			// Constant for Direction ClockWise	
	public static final int CCW = -1;			// Constant for Direction Counter Clockwise
	
	// ***************************************************
	//  Segment Class Methods
	//***************************************************
	
public Segment(){
	// Constructor
	type = 'N';
	startPt = new Point(0,0);
	ctrPt = new Point(0,0);
	endPt = new Point(0,0);
	len = 0;
	radius = 0;
	dir = 0;
	startAngle = 0;
	endAngle = 0;
	arcAngle = 0;
	startAngle = 0;
	endAngle = 0;
	vRatio = 0;
	vLeft = 0;
	vRight = 0;
}

//------------- Initialize Straight Line Segment ---------
public void strt(Point startPt1, Point endPt1){
	type = 'S';
	startPt = startPt1;
	ctrPt = new Point(0,0);
	endPt = endPt1;
	len = startPt1.dist(endPt1);
	radius = 0;
	dir = 1;
	startAngle = 0;
	endAngle = 0;
	arcAngle = 0;
	vRatio = 1;
	vLeft = 1;
	vRight = 1;
}

//------------- Initialize Arc Segment ---------
public void arc(Point startPt1, Point endPt1, Point ctrPt1, double radius1,  int dir1, double startAngle1){
	type = 'A';
	startPt = startPt1;
	ctrPt = ctrPt1;
	endPt = endPt1;
	radius = radius1;
	dir = dir1;
	startAngle = startAngle1;
	calcArcDetails();
}

//********** Calculate angle/length between two arcs points Function **************
public void calcArcDetails(){
	// startAhgle = robot hdg 0-360
	// dir > 0 means Arc is Clockwise from pt1 to pt2
	
	// Calculate Start and End Point Polar Angle
	Point tAngle1 = new Point();
	Point tAngle2 = new Point();
	tAngle1 = ctrPt.cartToPolar(startPt);	// returns hdg angle between 0-360
	tAngle2 = ctrPt.cartToPolar(endPt);		// returns hdg angle between 0-360
	if (dir == CCW){
		startAngle = Rmath.constrainDeg0To360(tAngle1.y + 90);
		endAngle =   Rmath.constrainDeg0To360(tAngle2.y + 90);				
		arcAngle = tAngle1.y - (tAngle2.y - (double)360);	
		arcAngle = Rmath.constrainDeg0To360(arcAngle);
		arcAngle = 360 - arcAngle;		
	} else {
		startAngle = Rmath.constrainDeg0To360(tAngle1.y - 90);
		endAngle =   Rmath.constrainDeg0To360(tAngle2.y - 90);
		arcAngle = tAngle1.y - (tAngle2.y - (double)360);
		arcAngle = Rmath.constrainDeg0To360(arcAngle);	
	}
	// Calculate Arc Segment Length
	len = ((2 * Math.PI * radius)/360) * arcAngle;

	// Calculate Radius to Velocity Ratios
	// Velocities are (-1) -> (+1)
	double rbtWheelbase = 28;		// distance between Ctr left Wheel and Ctr right Wheel
	double vR, vL, divisor;
	vR = radius + (rbtWheelbase /2);
	vL =  radius - (rbtWheelbase /2);
	if (Math.abs(vR) > Math.abs(vL))
		divisor = vR;
	else
		divisor = vL;
	vRight = vR / divisor;
	vLeft =  vL / divisor;
	if (vLeft == 0) vLeft = 0.000000001;		// prevent divide by zero
	vRatio = vRight / vLeft;
	return;
}

public void equals(Segment seg1){
	type = seg1.type;
	startPt = seg1.startPt;
	ctrPt = seg1.ctrPt;
	endPt = seg1.endPt;
	len = seg1.len;
	radius = seg1.radius;
	dir = seg1.dir;
	startAngle = seg1.startAngle;
	endAngle = seg1.endAngle;
	arcAngle = seg1.arcAngle;
	vRatio = seg1.vRatio;
	vLeft = seg1.vLeft;
	vRight = seg1.vRight;
}

public void print(){
	System.out.println(" Segment Type=" + type);
	System.out.println(" Segment startPt" + startPt.toString());
	System.out.println(" Segment ctrPt" + ctrPt.toString());
	System.out.println(" Segment endPt" + endPt.toString());	
	System.out.println(" Segment len=" + len + "  radius=" + radius + "  dir=" + dir);	
	System.out.println(" Segment startAngle=" + startAngle + "  endAngle=" + endAngle + "  arcAngle=" + arcAngle);	
	System.out.println(" Segment vRatio=" + vRatio + " vLeft=" + vLeft + "  vRight=" + vRight);
}

}
