package robot.utils;

/**
 * @author Paul
 */
public class Position {
    public double x ;
    public double y ;
    public double orientCW ;
    public String name;
    
    public Position() {}
    
    public Position(Position posn) {
        x = posn.x ;
        y = posn.y ;
        orientCW = posn.orientCW ;
        name = posn.name ;
    }

    public Position(double x, double y, double orient, String name) {
        this.x = x ;
        this.y = y ;
        this.orientCW = orient ;
        this.name = name;
    }
}
