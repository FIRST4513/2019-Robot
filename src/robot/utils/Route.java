package robot.utils;

//import java.awt.List;
//import java.lang.Math;
//import java.util.ArrayList;
//import robot.utils.*;

public class Route {
	
	// --- Route Data Elements ---
	public int segCnt;				// Count of active segments
	public Segment[] seg;
	private final int ROUTESIZE = 10;
	
 // ***************************************************
 //               Route Class Methods
 // ***************************************************
 public Route(){
	seg = new Segment[ROUTESIZE];
 	int i = 0;
 	while (i < ROUTESIZE){
 		seg[i] = new Segment();
 		//seg[i].type = 'N';
 		i++;}
 	segCnt = 0;			// Zero current active segments in route
 }

 public void setSeg(int i, Segment seg1){
 	seg[i] = seg1;
 }

 public Segment getSeg(int i){
 	return (seg[i]);
 }

 public void clearSeg(int i){
 	//seg[i].type = 'N';
 	segCnt -= 1;			// Zero current active segments in route
 	if (segCnt < 0) segCnt = 0;
 }

 public void clearAllSeg(){
 	int i = 0;
 	while (i < ROUTESIZE){
 		clearSeg(i);
 		i++;
 	}
 	segCnt = 0;
 }

 public boolean append(Segment seg1){
 	if (segCnt >= ROUTESIZE) return false;	// Route Full
 	if (segCnt < 0) segCnt = 0;		// This shouldnt ever happen ...
 	seg[segCnt] = seg1;				// Add segment to route
 	segCnt++;
 	return true;
 }

 public void equals(Route rt1){
 	int i = 0;
 	while ( i < ROUTESIZE){
 		seg[i] = rt1.seg[i];
 		i++;
 	}
 	segCnt = rt1.segCnt;
 }

 public void print(){
	 	int i = 0;
	 	System.out.println(" Route contains " + segCnt + " Segments");
	 	while ( i < segCnt){
	 		System.out.println("Route Segment " + i);
	 		seg[i].print();
	 		i++;
	 	}
	 }
 
 
 
 
}
