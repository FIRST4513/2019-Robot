package robot.utils;

import robot.utils.Position;
//import java.lang.Math;
//import robot.utils.Rmath;

public class Map {
    
    // y boundaries for targets
    private static final int NUM_BOUNDARY_LINES = 4 ;
    private static final double[] Y_BOUNDARIES = {14.5, 18.5, 21.0, 22.7} ;

    //private static enum Side {LEFT, RIGHT};
    //private static Side side;
    private static int side = 0;
    // -------------------- Cargo Ship Positions --------------------
    private static String cargoShip_Name[][]      = new String[2][4];     // [Left,Right],[Front, Near, Ctr, Far]
    private static double cargoShip_Vision_X[][]  = new double[2][4];     // [Left,Right],[Front, Near, Ctr, Far]
    private static double cargoShip_Vision_Y[][]  = new double[2][4];     // [Left,Right],[Front, Near, Ctr, Far]
    private static double cargoShip_Tgt_X[][]     = new double[2][4];     // [Left,Right],[Front, Near, Ctr, Far]
    private static double cargoShip_Tgt_Y[][]     = new double[2][4];     // [Left,Right],[Front, Near, Ctr, Far]
    private static double cargoShip_Tgt_O[][]     = new double[2][4];     // [Left,Right],[Front, Near, Ctr, Far]

    // -------------------- Rocket Positions --------------------
    private static String rocket_Name[][]         = new String[2][4];     // [Left,Right],[Near, Ctr, Far, Far]
    private static double rocket_Vision_X[][]     = new double[2][4];     // [Left,Right],[Near, Ctr, Far, Far]
    private static double rocket_Vision_Y[][]     = new double[2][4];     // [Left,Right],[Near, Ctr, Far, Far]
    private static double rocket_Tgt_X[][]        = new double[2][4];     // [Left,Right],[Near, Ctr, Far, Far]
    private static double rocket_Tgt_Y[][]        = new double[2][4];     // [Left,Right],[Near, Ctr, Far, Far]
    private static double rocket_Tgt_O[][]        = new double[2][4];     // [Left,Right],[Near, Ctr, Far, Far]

    // -------------------- Loading Station Positions --------------------
    private static String loadingSt_Name[]         = new String[2];     // [Left,Right]
    private static double loadingSt_Vision_X[]     = new double[2];     // [Left,Right]
    private static double loadingSt_Vision_Y[]     = new double[2];     // [Left,Right]
    public static double loadingSt_Tgt_X[]        = new double[2];     // [Left,Right]
    public static double loadingSt_Tgt_Y[]        = new double[2];     // [Left,Right]
    private static double loadingSt_Tgt_O[]        = new double[2];     // [Left,Right]
    
    // --- Map Data Elements ---

    public static final double robot_Len_WB = 19.25;   // center axle to front of bumber distance
    public static final double robot_Len_WOB = 16.0;   // center axle to front of chassi without bumber

    // -------------- Starting Positions -------------------
    public final static String StartingPos_Right_FacingRight_Name = "Starting Position Right Facing Right 1st Level";
    public final static double StartingPos_Right_FacingRight_X = 45.25;
    public final static double StartingPos_Right_FacingRight_Y = 66.0;
    public final static double StartingPos_Right_FacingRight_O = 90.0;

    public final static String StartingPos_Right_FacingFwd_Name = "Starting Position Right Facing Fwd 1st Level";
    public final static double StartingPos_Right_FacingFwd_X = 45.75;
    public final static double StartingPos_Right_FacingFwd_Y = 68.125;
    public final static double StartingPos_Right_FacingFwd_O = 0.0;

    public final static String StartingPos_Ctr_FacingFwd_Name = "Starting Position Center Facing Forward 1st Level";
    public final static double StartingPos_Ctr_FacingFwd_X = 0.0;
    public final static double StartingPos_Ctr_FacingFwd_Y = 68.125;
    public final static double StartingPos_Ctr_FacingFwd_O = 0.0;

    public final static String StartingPos_Left_FacingFwd_Name = "Starting Position Left Facing Forward 1st Level";
    public final static double StartingPos_Left_FacingFwd_X = -45.75;
    public final static double StartingPos_Left_FacingFwd_Y = 68.125;
    public final static double StartingPos_Left_FacingFwd_O = 0.0;

    public final static String StartingPos_Left_FacingLeft_Name = "Starting Position Left Facing Left 1st Level";
    public final static double StartingPos_Left_FacingLeft_X = -45.25;
    public final static double StartingPos_Left_FacingLeft_Y = 66.0;
    public final static double StartingPos_Left_FacingLeft_O = -90.0;

    // ----------------- Starting Position Level 2 -----------------
    public final static String StartingPos_Level2_Left_Fwd_FacingLeft_Name = "Starting Position Left Facing Fwd 2nd Level";
    public final static double StartingPos_Level2_Left_Fwd_FacingLeft_X = -45.25;
    public final static double StartingPos_Level2_Left_Fwd_FacingLeft_Y = 44.0;
    public final static double StartingPos_Level2_Left_Fwd_FacingLeft_O = 0.0;

    public final static String StartingPos_Level2_Right_Fwd_FacingLeft_Name = "Starting Position Right Facing Fwd 2nd Level";
    public final static double StartingPos_Level2_Right_Fwd_FacingLeft_X = 45.25;
    public final static double StartingPos_Level2_Right_Fwd_FacingLeft_Y = 44.0;
    public final static double StartingPos_Level2_Right_Fwd_FacingLeft_O = 0.0;

    // --------------- Loading Station Positions ----------------
    public final static String LoadingStation_Right_Name = "Loading Station Right";
    public final static double LoadingStation_Right_Vision_X = 132.75;
    public final static double LoadingStation_Right_Vision_Y = 0.0;
    public final static double LoadingStation_Right_Tgt_X = LoadingStation_Right_Vision_X;
    public final static double LoadingStation_Right_Tgt_Y = LoadingStation_Right_Vision_Y + robot_Len_WOB;
    public final static double LoadingStation_Right_Tgt_O = 180.0;

    public final static String LoadingStation_Left_Name = "Loading Station Left";
    public final static double LoadingStation_Left_Vision_X = 0.0;
    public final static double LoadingStation_Left_Vision_Y = 0.0;
    public final static double LoadingStation_Left_Tgt_X = LoadingStation_Left_Vision_X;
    public final static double LoadingStation_Left_Tgt_Y = LoadingStation_Left_Vision_Y + robot_Len_WOB;
    public final static double LoadingStation_Left_Tgt_O = 180.0;


    // -------------------- Cargoship Positions ------------------------
    public final static String CargoShip_Right_Front_Name = "CargoShip Right Front";
    public final static double CargoShip_Right_Front_Vision_X = 11.0;
    public final static double CargoShip_Right_Front_Vision_Y = 220.5;
    public final static double CargoShip_Right_Front_Tgt_X = CargoShip_Right_Front_Vision_X;
    public final static double CargoShip_Right_Front_Tgt_Y =  CargoShip_Right_Front_Vision_Y - robot_Len_WOB ;
    public final static double CargoShip_Right_Front_Tgt_O = 0.0;


    public final static String CargoShip_Right_Near_Name = "CargoShip Right Near";
    public final static double CargoShip_Right_Near_Vision_X = 27.75;
    public final static double CargoShip_Right_Near_Vision_Y = 261.0;
    public final static double CargoShip_Right_Near_Tgt_X = CargoShip_Right_Near_Vision_X + robot_Len_WOB ;
    public final static double CargoShip_Right_Near_Tgt_Y = CargoShip_Right_Near_Vision_Y;
    public final static double CargoShip_Right_Near_Tgt_O = -90.0;

    
    public final static String CargoShip_Right_Ctr_Name = "CargoShip Right Center";
    public final static double CargoShip_Right_Ctr_Vision_X = 27.75;
    public final static double CargoShip_Right_Ctr_Vision_Y = 282.75;
    public final static double CargoShip_Right_Ctr_Tgt_X = CargoShip_Right_Ctr_Vision_X + robot_Len_WOB ;
    public final static double CargoShip_Right_Ctr_Tgt_Y = CargoShip_Right_Ctr_Vision_Y;
    public final static double CargoShip_Right_Ctr_Tgt_O = -90.0;


    public final static String CargoShip_Right_Far_Name = "CargoShip Right Far";
    public final static double CargoShip_Right_Far_Vision_X = 27.75;
    public final static double CargoShip_Right_Far_Vision_Y = 304.5;
    public final static double CargoShip_Right_Far_Tgt_X = CargoShip_Right_Far_Vision_X + robot_Len_WOB ;
    public final static double CargoShip_Right_Far_Tgt_Y = CargoShip_Right_Far_Vision_Y;
    public final static double CargoShip_Right_Far_Tgt_O = -90.0;


    public final static String CargoShip_Left_Front_Name = "CargoShip Left Front";
    public final static double CargoShip_Left_Front_Vision_X = -11.0;
    public final static double CargoShip_Left_Front_Vision_Y = 220.5;
    public final static double CargoShip_Left_Front_Tgt_X = CargoShip_Left_Front_Vision_X;
    public final static double CargoShip_Left_Front_Tgt_Y =CargoShip_Left_Front_Vision_Y - robot_Len_WOB ;
    public final static double CargoShip_Left_Front_Tgt_O = 0.0;


    public final static String CargoShip_Left_Near_Name = "CargoShip Left Near";
    public final static double CargoShip_Left_Near_Vision_X = -27.75;
    public final static double CargoShip_Left_Near_Vision_Y = 261;
    public final static double CargoShip_Left_Near_Tgt_X = CargoShip_Left_Near_Vision_X - robot_Len_WOB ;
    public final static double CargoShip_Left_Near_Tgt_Y = CargoShip_Left_Near_Vision_Y;
    public final static double CargoShip_Left_Near_Tgt_O = 90.0;


    public final static String CargoShip_Left_Ctr_Name = "CargoShip Left Center";
    public final static double CargoShip_Left_Ctr_Vision_X = -27.75;
    public final static double CargoShip_Left_Ctr_Vision_Y = 282.75;
    public final static double CargoShip_Left_Ctr_Tgt_X = CargoShip_Left_Ctr_Vision_X - robot_Len_WOB ;
    public final static double CargoShip_Left_Ctr_Tgt_Y = CargoShip_Left_Ctr_Vision_Y;
    public final static double CargoShip_Left_Ctr_Tgt_O = 90.0;


    public final static String CargoShip_Left_Far_Name = "CargoShip Left Far";
    public final static double CargoShip_Left_Far_Vision_X = -27.75;
    public final static double CargoShip_Left_Far_Vision_Y = 304.5;
    public final static double CargoShip_Left_Far_Tgt_X = CargoShip_Left_Far_Vision_X - robot_Len_WOB ;
    public final static double CargoShip_Left_Far_Tgt_Y = CargoShip_Left_Far_Vision_Y;
    public final static double CargoShip_Left_Far_Tgt_O = 90.0;


    // ----------------- Rocket Positions -----------------------

    public final static String Rocket_Right_Near_Name = "Rocket Right Near";
    public final static double Rocket_Right_Near_Vision_X = 142.75;
    public final static double Rocket_Right_Near_Vision_Y = 214.125;
    public final static double Rocket_Right_Near_Tgt_X = 133.0;
    public final static double Rocket_Right_Near_Tgt_Y = 196.625;
    public final static double Rocket_Right_Near_Tgt_O = 29.0;


    public final static String Rocket_Right_Ctr_Name = "Rocket Right Center";
    public final static double Rocket_Right_Ctr_Vision_X = 133.75;
    public final static double Rocket_Right_Ctr_Vision_Y = 229;
    public final static double Rocket_Right_Ctr_Tgt_X = Rocket_Right_Ctr_Vision_X - robot_Len_WB ;
    public final static double Rocket_Right_Ctr_Tgt_Y = Rocket_Right_Ctr_Vision_Y;
    public final static double Rocket_Right_Ctr_Tgt_O = 90.0;


    public final static String Rocket_Right_Far_Name = "Rocket Right Far";
    public final static double Rocket_Right_Far_Vision_X = 142.75;
    public final static double Rocket_Right_Far_Vision_Y = 244.125;
    public final static double Rocket_Right_Far_Tgt_X = 133.0;
    public final static double Rocket_Right_Far_Tgt_Y = 261.625;
    public final static double Rocket_Right_Far_Tgt_O = 119.0;


    public final static String Rocket_Left_Near_Name = "Rocket Left Near";
    public final static double Rocket_Left_Near_Vision_X = 0-142.75;
    public final static double Rocket_Left_Near_Vision_Y = 214.125;
    public final static double Rocket_Left_Near_Tgt_X = -133.0;
    public final static double Rocket_Left_Near_Tgt_Y = 196.625;
    public final static double Rocket_Left_Near_Tgt_O = -29.0;


    public final static String Rocket_Left_Ctr_Name = "Rocket Left Center";
    public final static double Rocket_Left_Ctr_Vision_X = -133.75;
    public final static double Rocket_Left_Ctr_Vision_Y = 229.0;
    public final static double Rocket_Left_Ctr_Tgt_X = Rocket_Left_Ctr_Vision_X + robot_Len_WB ;
    public final static double Rocket_Left_Ctr_Tgt_Y = Rocket_Left_Ctr_Vision_Y;
    public final static double Rocket_Left_Ctr_Tgt_O = -90.0;


    public final static String Rocket_Left_Far_Name = "Rocket Left Far";
    public final static double Rocket_Left_Far_Vision_X = -142.75;
    public final static double Rocket_Left_Far_Vision_Y = 244.125;
    public final static double Rocket_Left_Far_Tgt_X = -133.0;
    public final static double Rocket_Left_Far_Tgt_Y = 261.625;
    public final static double Rocket_Left_Far_Tgt_O = -119.0;

	public Map(){
        // Constructor with initial values
        init_Position_Tables();
    }


    // Figure out target location and orientation based on bot loc and orient 
    public static Position getRobotPosBasedOnVision( double heading, double distance, double gyroYaw) {
        // Calculate robot position on field from vision target hdg and distance and robots current gyro heading
        // This can be improved by useing vision heading and distance and target skew
        //
        Position currPosn = new Position() ;
        return currPosn;
    }

    // Figure out target location and orientation based on bot loc and orient 
    public static Position getTargetPosBasedOnRobotPosition(Position botPosn) {
        // Get the target posn and orient from the bot posn and orient
        int index ;
        double targX, targY, targOrient ;
        String targName;

        if (botPosn.x < 0) side = 0;        // We're on Left  side of field
        else               side = 1;        // We're on Right side of field

        for (index=0 ; index<NUM_BOUNDARY_LINES ; index++) {
            // determines bots index = zone position on field
            if (botPosn.y<=Y_BOUNDARIES[index])
                break ;
        }

        if (botPosn.orientCW < 0.0) {
            // We're looking at cargo ship
            // index 0  = front, 1 = near,  2 = ctr, 3 = far
            targX       = cargoShip_Tgt_X[side][index];
            targY       = cargoShip_Tgt_Y[side][index];
            targOrient  = cargoShip_Tgt_O[side][index];
            targName    = cargoShip_Name[side][index] ;
        } else {
            // We're looking at rocket
            // index 0 = near, 1 = Ctr, 2 = far, 3 = far
            targX       = rocket_Tgt_X[side][index];
            targY       = rocket_Tgt_Y[side][index];
            targOrient  = rocket_Tgt_O[side][index];
            targName    = rocket_Name[side][index]; 
        }      
        
        // Subtract the bot position on the field from the target posn
        // in order to make it relative to the bot
        // targX -= botPosn.x ;
        // targY -= botPosn.y ;
        
        Position targPosn = new Position() ;
        targPosn.x = targX ;
        targPosn.y = targY ;
        targPosn.orientCW = targOrient ;
        targPosn.name = targName ;
        //System.out.println("Target is " + TARGET_NAME[nameIndex]);
        return targPosn ;
    }

    
    // ------------------------- Init Tables -----------------------------
    public int calcZone(Position pos){
        // Zone 1 = Loading Station Left
        // Zone 2 = Front of cargo ship Left
        // Zone 3 = Side of cargo ship Left
        // Zone 4 = Rocket Near Left
        // Zone 5 = Roket Ctr Left
        // Zone 6 = Rocket Far Left

        // Zone 11 = Loading Station Right
        // Zone 12 = Front of cargo ship Right
        // Zone 13 = Side of cargo ship Right
        // Zone 14 = Rocket Near Right
        // Zone 15 = Roket Ctr Right
        // Zone 16 = Rocket Far Right

        // Line A line formula is y = 1.37134 x + 153.12
        // Line B line formula is y =
        return 2;
    }    
    
    // ------------------------- Init Tables -----------------------------
    private void init_Position_Tables(){
        init_Rocket_Position_Table();
        init_CargoShip_Position_Table();
        init_LoadingSt_Position_Table();

    }

    private void init_LoadingSt_Position_Table(){
        // -------------------- Loading Station Positions --------------------
        // Table definitions [Left,Right]
        loadingSt_Name[0]         = LoadingStation_Left_Name;
        loadingSt_Vision_X[0]     = LoadingStation_Left_Vision_X;
        loadingSt_Vision_Y[0]     = LoadingStation_Left_Vision_Y;
        loadingSt_Tgt_X[0]        = LoadingStation_Left_Tgt_X;
        loadingSt_Tgt_Y[0]        = LoadingStation_Left_Tgt_Y;
        loadingSt_Tgt_O[0]        = LoadingStation_Left_Tgt_O;

        loadingSt_Name[1]         = LoadingStation_Right_Name;
        loadingSt_Vision_X[1]     = LoadingStation_Right_Vision_X;
        loadingSt_Vision_Y[1]     = LoadingStation_Right_Vision_Y;
        loadingSt_Tgt_X[1]        = LoadingStation_Right_Tgt_X;
        loadingSt_Tgt_Y[1]        = LoadingStation_Right_Tgt_Y;
        loadingSt_Tgt_O[1]        = LoadingStation_Right_Tgt_O;
    }

    private void init_Rocket_Position_Table(){
        // -------------------- Rocket Positions --------------------
        // Table definitions [Left,Right], [Near, Ctr, Far, Far]
        rocket_Name[0][0]         = Rocket_Left_Near_Name;
        rocket_Name[0][1]         = Rocket_Left_Ctr_Name;
        rocket_Name[0][2]         = Rocket_Left_Far_Name;
        rocket_Name[0][3]         = Rocket_Left_Far_Name;

        rocket_Name[1][0]         = Rocket_Right_Near_Name;
        rocket_Name[1][1]         = Rocket_Right_Ctr_Name;
        rocket_Name[1][2]         = Rocket_Right_Far_Name;
        rocket_Name[1][3]         = Rocket_Right_Far_Name;

        rocket_Vision_X[0][0]     = Rocket_Left_Near_Vision_X;
        rocket_Vision_X[0][1]     = Rocket_Left_Ctr_Vision_X;
        rocket_Vision_X[0][2]     = Rocket_Left_Far_Vision_X;
        rocket_Vision_X[0][3]     = Rocket_Left_Far_Vision_X;

        rocket_Vision_X[1][0]     = Rocket_Right_Near_Vision_X;
        rocket_Vision_X[1][1]     = Rocket_Right_Ctr_Vision_X;
        rocket_Vision_X[1][2]     = Rocket_Right_Far_Vision_X;
        rocket_Vision_X[1][3]     = Rocket_Right_Far_Vision_X;

        rocket_Vision_Y[0][0]     = Rocket_Left_Near_Vision_Y;
        rocket_Vision_Y[0][1]     = Rocket_Left_Ctr_Vision_Y;
        rocket_Vision_Y[0][2]     = Rocket_Left_Far_Vision_Y;
        rocket_Vision_Y[0][3]     = Rocket_Left_Far_Vision_Y;

        rocket_Vision_Y[1][0]     = Rocket_Right_Near_Vision_Y;
        rocket_Vision_Y[1][1]     = Rocket_Right_Ctr_Vision_Y;
        rocket_Vision_Y[1][2]     = Rocket_Right_Far_Vision_Y;
        rocket_Vision_Y[1][3]     = Rocket_Right_Far_Vision_Y;

        rocket_Tgt_X[0][0]        = Rocket_Left_Near_Tgt_X;
        rocket_Tgt_X[0][1]        = Rocket_Left_Ctr_Tgt_X;
        rocket_Tgt_X[0][2]        = Rocket_Left_Far_Tgt_X;
        rocket_Tgt_X[0][3]        = Rocket_Left_Far_Tgt_X;

        rocket_Tgt_X[1][0]        = Rocket_Right_Near_Tgt_X;
        rocket_Tgt_X[1][1]        = Rocket_Right_Ctr_Tgt_X;
        rocket_Tgt_X[1][2]        = Rocket_Right_Far_Tgt_X;
        rocket_Tgt_X[1][3]        = Rocket_Right_Far_Tgt_X;

        rocket_Tgt_Y[0][0]        = Rocket_Left_Near_Tgt_Y;
        rocket_Tgt_Y[0][1]        = Rocket_Left_Ctr_Tgt_Y;
        rocket_Tgt_Y[0][2]        = Rocket_Left_Far_Tgt_Y;
        rocket_Tgt_Y[0][3]        = Rocket_Left_Far_Tgt_Y;

        rocket_Tgt_Y[1][0]        = Rocket_Right_Near_Tgt_Y;
        rocket_Tgt_Y[1][1]        = Rocket_Right_Ctr_Tgt_Y;
        rocket_Tgt_Y[1][2]        = Rocket_Right_Far_Tgt_Y;
        rocket_Tgt_Y[1][3]        = Rocket_Right_Far_Tgt_Y;

        rocket_Tgt_O[0][0]        = Rocket_Left_Near_Tgt_O;
        rocket_Tgt_O[0][1]        = Rocket_Left_Ctr_Tgt_O;
        rocket_Tgt_O[0][2]        = Rocket_Left_Far_Tgt_O;
        rocket_Tgt_O[0][3]        = Rocket_Left_Far_Tgt_O;

        rocket_Tgt_O[1][0]        = Rocket_Right_Near_Tgt_O;
        rocket_Tgt_O[1][1]        = Rocket_Right_Ctr_Tgt_O;
        rocket_Tgt_O[1][2]        = Rocket_Right_Far_Tgt_O;
        rocket_Tgt_O[1][3]        = Rocket_Right_Far_Tgt_O;
    }

    private void init_CargoShip_Position_Table(){
        // -------------------- Cargo Ship Positions --------------------

       cargoShip_Name[0][0]     = CargoShip_Left_Front_Name;
       cargoShip_Name[0][1]     = CargoShip_Left_Near_Name;
       cargoShip_Name[0][2]     = CargoShip_Left_Ctr_Name;
       cargoShip_Name[0][3]     = CargoShip_Left_Far_Name;

       cargoShip_Name[1][0]     = CargoShip_Right_Front_Name;
       cargoShip_Name[1][1]     = CargoShip_Right_Near_Name;
       cargoShip_Name[1][2]     = CargoShip_Right_Ctr_Name;
       cargoShip_Name[1][3]     = CargoShip_Right_Far_Name;

       cargoShip_Vision_X[0][0] = CargoShip_Left_Front_Vision_X;
       cargoShip_Vision_X[0][1] = CargoShip_Left_Near_Vision_X;
       cargoShip_Vision_X[0][2] = CargoShip_Left_Ctr_Vision_X;
       cargoShip_Vision_X[0][3] = CargoShip_Left_Far_Vision_X;

       cargoShip_Vision_X[1][0] = CargoShip_Right_Front_Vision_X;
       cargoShip_Vision_X[1][1] = CargoShip_Right_Near_Vision_X;
       cargoShip_Vision_X[1][2] = CargoShip_Right_Ctr_Vision_X;
       cargoShip_Vision_X[1][3] = CargoShip_Right_Far_Vision_X;

       cargoShip_Vision_Y[0][0] = CargoShip_Left_Front_Vision_Y;
       cargoShip_Vision_Y[0][1] = CargoShip_Left_Near_Vision_Y;
       cargoShip_Vision_Y[0][2] = CargoShip_Left_Ctr_Vision_Y;
       cargoShip_Vision_Y[0][3] = CargoShip_Left_Far_Vision_Y;

       cargoShip_Vision_Y[1][0] = CargoShip_Right_Front_Vision_Y;
       cargoShip_Vision_Y[1][1] = CargoShip_Right_Near_Vision_Y;
       cargoShip_Vision_Y[1][2] = CargoShip_Right_Ctr_Vision_Y;
       cargoShip_Vision_Y[1][3] = CargoShip_Right_Far_Vision_Y;

       cargoShip_Tgt_X[0][0]    = CargoShip_Left_Front_Tgt_X;
       cargoShip_Tgt_X[0][1]    = CargoShip_Left_Near_Tgt_X;
       cargoShip_Tgt_X[0][2]    = CargoShip_Left_Ctr_Tgt_X;
       cargoShip_Tgt_X[0][3]    = CargoShip_Left_Far_Tgt_X;

       cargoShip_Tgt_X[1][0]    = CargoShip_Right_Front_Tgt_X;
       cargoShip_Tgt_X[1][1]    = CargoShip_Right_Near_Tgt_X;
       cargoShip_Tgt_X[1][2]    = CargoShip_Right_Ctr_Tgt_X;
       cargoShip_Tgt_X[1][3]    = CargoShip_Right_Far_Tgt_X;

       cargoShip_Tgt_Y[0][0]    = CargoShip_Left_Front_Tgt_Y;
       cargoShip_Tgt_Y[0][1]    = CargoShip_Left_Near_Tgt_Y;
       cargoShip_Tgt_Y[0][2]    = CargoShip_Left_Ctr_Tgt_Y;
       cargoShip_Tgt_Y[0][3]    = CargoShip_Left_Far_Tgt_Y;

       cargoShip_Tgt_Y[1][0]    = CargoShip_Right_Front_Tgt_Y;
       cargoShip_Tgt_Y[1][1]    = CargoShip_Right_Near_Tgt_Y;
       cargoShip_Tgt_Y[1][2]    = CargoShip_Right_Ctr_Tgt_Y;
       cargoShip_Tgt_Y[1][3]    = CargoShip_Right_Far_Tgt_Y;

       cargoShip_Tgt_O[0][0]    = CargoShip_Left_Front_Tgt_O;
       cargoShip_Tgt_O[0][1]    = CargoShip_Left_Near_Tgt_O;
       cargoShip_Tgt_O[0][2]    = CargoShip_Left_Ctr_Tgt_O;
       cargoShip_Tgt_O[0][3]    = CargoShip_Left_Far_Tgt_O;

       cargoShip_Tgt_O[1][0]    = CargoShip_Right_Front_Tgt_O;
       cargoShip_Tgt_O[1][1]    = CargoShip_Right_Near_Tgt_O;
       cargoShip_Tgt_O[1][2]    = CargoShip_Right_Ctr_Tgt_O;
       cargoShip_Tgt_O[1][3]    = CargoShip_Right_Far_Tgt_O;
  
    }
}