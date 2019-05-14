package robot.utils;

import java.lang.Math;
import robot.utils.Rmath;

public class Point {
	
	// --- Point Data Elements ---
	public double x;
	public double y;
	
	
	//---- Point Methods -----
	public Point(double x1, double y1){
		// Constructor with initial values
		x = x1;
		y = y1;
	}
	
	public Point(){
		// constructor with no parameters
		x = 0;
		y = 0;
	}
	
	public void set(double x1, double y1){
		// Set self equal to x1, y1 point values
		x = x1;
		y = y1;	
	}
	
	public void set(Point pt){
		// Set self equal to pt
		x = pt.x;
		y = pt.y;
	}
	
	public double dist(Point pt){
		// return the Euclidian distance between self and pt
	    double dx = x - pt.x;
	    double dy = y - pt.y;
	    return (Math.sqrt((dx * dx) + (dy * dy)));
	}
	
	public void reset(){
		// Reset x&y to zero
	    x = 0;
	    y = 0;
	}

	// *************************** Cartesian To Polar Conversion **********************
	public Point cartToPolar(){
		// Convert Cart. Coord Cart.Coord(x,y) to Polar Cord Point(radius, angle)
		double x1, y1, radius, angle;
		x1 = x;
		y1 = y;
		radius = Math.sqrt((x1*x1) + (y1*y1));
		angle = Rmath.mAtan(y1,x1);
		Point retPt = new Point(radius, angle);
		return retPt;
	}
	
	public Point cartToPolar(Point newPt){
		double x1, y1, radius, angle;
		x1 = newPt.x - x;
		y1 = newPt.y - y;
	    radius = Math.sqrt((x1*x1) + (y1*y1));
	    angle = Rmath.mAtan(y1,x1);
		Point retPt = new Point(radius, angle);
		return (retPt);	
	}
	
	// *************************** Polar to Cartesian Coversion ***********************
	public Point polarToCart(double radius, double angle){
		// Convert Polar Coord radius, angle to Cart.Coord(x,y)
		double x,y;
		x = radius * Rmath.mCos(angle);
		y = radius * Rmath.mSin(angle);
		Point retPt = new Point(x,y);
		return (retPt);
	}
	
	public Point polarToCart(double radius, double angle, Point pt){
		// Takes in a radius, angle and returns a cartesian point relative to Point pt
	    double x = radius * Rmath.mCos(angle);
	    double y = radius * Rmath.mSin(angle);
	    Point retPt = new Point(pt.x+x, pt.y+y);
	    return (retPt);
	}

	// *************************** Operator Overloads (sort of) ***********************
	public boolean eq(Point pt){
		// Test for Equality of 2 points
		if ((x == pt.x) && (y == pt.y)) return true;
		return false;		
	};
	
	public boolean ne(Point pt){
		// Test for not equal of 2 points
		if ((x == pt.x) && (y == pt.y)) return false;
		return true;
	}
	public void equals(Point pt){
		// Assignment
		x = pt.x;
		y = pt.y;
		return;
	}
	public void add(Point pt){
		// Adds 2 points together
		// return a new point found by adding self and p. This method is
	    // called by e.g. p+q for points p and q
		x += pt.x;
		y += pt.y;
		return;
	}
	
	public String toString(){
		// Return String of point
		String retString = "";
		retString = "(x=" + x + " y=" + y + ")";
		return retString;
}
	
}