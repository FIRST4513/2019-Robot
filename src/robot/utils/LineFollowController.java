package robot.utils;

import java.awt.image.BufferedImage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.Robot;

// Implemented as a singleton because there should probably not be more than 
// a single instance at any given time (following two lines at the same
// time isn't physically possible). It also allows the Animation class to 
// access it directly, which is more convenient in simulation.
// Using lazy instantiation so as not to waste any memory in cases where it 
// isn't used at all.
public final class LineFollowController
{
    // this struct is used to return motor controls
    public class MotorControlStruct {
        public double left = 0.0 ;
        public double right = 0.0 ;
    }
    
    enum LineFollowState {NEEDS_INIT, LINE, BRAKE, DONE} ;
    
    // PID params
    private double KP = 0.82 ;
    private double KD = 0.013 ;
    private double KI = 0.00001 ;
    // when hit a red area, put on brakes
    // brakes intensity is relative to velocity
    private double  BRAKE_PER_VEL_IN_FTPS = 0.1 ; 
    
    // sensor params
    private final int    N_SENSORS = 5 ;   
    private final double SENSOR_POS_MIDINDEX = ((double)N_SENSORS-1.0)/2.0 ;
    // how many ft is sensor array in front of the center of mass
    static public final double SENSOR_STANDOFF_FT = 1.5 ; 
    // spacing between sensors is calculated from the line width 
    private final double SENSORS_PER_LINE = 1.5 ;
    private double mSensorSpacingFt ; 
    
    // sensor locations (calculated from the above values
    private final double[] mSensX = new double[N_SENSORS] ;
    private final double[] mSensY = new double[N_SENSORS] ;
    private double mLastSensorVal = (double)(N_SENSORS-1) / 2.0 ;
    private double mLastError = 0.0 ;
    private double mSumError = 0.0 ;
    
    // simulated region
    private BufferedImage mImage = null ;
    private double mImagePixW = 0 ;
    private double mImagePixH = 0 ; 
    private double mPixPerIn  ;                   // scale factor
    private final int PIX_THRESH = 0x808080 ;   // threshold for white/black 
   
    // baseline acceleration and desired ending orientation
    private double  mBaseAccel ;

    private LineFollowState mState = LineFollowState.NEEDS_INIT ;

    // singleton instance
    private static LineFollowController instance = null ;
    
    // empty constructor 
    private LineFollowController() {}
    
    // note that this version of lazy initialization is not thread-safe
    public static LineFollowController getInstance() {
        if (instance==null) 
            instance = new LineFollowController() ;
        return instance ;
    }    
    
    public void init(
            double baseAccel,     // base drive power
            BufferedImage img,    // image of path
            double pix_per_in,    // image scale in pix/in
            double linewinches,   // width of the line in inches
            double endTheta,      // final desired angle (Double.Nan if don't care)
            double x, double y,   // current location (usually 0,0)
            double orientDegCCW   // current abs orientation on playfield
            )  
    {
        KP = SmartDashboard.getNumber("LINEFOLLOW_KP", KP) ;
        KD = SmartDashboard.getNumber("LINEFOLLOW_KD", KD) ;
        BRAKE_PER_VEL_IN_FTPS = SmartDashboard.getNumber("BRAKE_PER_VEL_IN_FTPS", BRAKE_PER_VEL_IN_FTPS) ; 
        
        mBaseAccel = baseAccel ;
        
        mImage = img ;
        if (img != null) {
            mImagePixW = (int)img.getWidth() ;
            mImagePixH = (int)img.getHeight() ;        	
        }
        
        this.mPixPerIn = pix_per_in ;
        mSensorSpacingFt = linewinches / 12.0 / SENSORS_PER_LINE  ;

        // init line reading to middle of sensor array
        mLastSensorVal = (double)(N_SENSORS-1) / 2.0 ;
        
        // init PID memory
        mLastError = 0.0 ;
        mSumError = 0.0 ;
        
        // make sure start out with valid sensor locations 
        // in case they are queried before the controller has been started
        upDateSensorLocs(x, y, orientDegCCW) ;
        
        // start out done
        mState = LineFollowState.DONE ;
    }
    
    public void start() {
        if (mState != LineFollowState.DONE) {
            throw new Error("cannot start LineFollowController because "
                            + "it is busy. Consider calling init first.") ;
        }
        else {
            mState = LineFollowState.LINE ;
        }
    }
    
    public void stop() {
        mState = LineFollowState.DONE ;
    }    

    public MotorControlStruct update(double x, double y, 
            	double orientDegCCW, double velFtPerSec, double elapsed) {
        MotorControlStruct ctrl = new MotorControlStruct() ;
        switch (mState) {
            case LINE:
                // transition to brake mode happens inside here:
                ctrl = lineFollowControlUpdate(x, y, orientDegCCW, 
                                velFtPerSec, elapsed) ;
                break ;
            case BRAKE:
                if (velFtPerSec<0.2) {
                    mState = LineFollowState.DONE ;
                }
                break ;
            case DONE:
                break ;
            case NEEDS_INIT:
            	break;
            default:
            	break;
        }
        return ctrl ;
    }

    private MotorControlStruct lineFollowControlUpdate(double x, double y, 
            		double absThetaDegCCW, double velFtPerSec, double dT) {
        // upDate sensor locations so can read if over line
        upDateSensorLocs(x, y, absThetaDegCCW) ;
        
        // convert sensor locs to pixel locs and get pixel value
        boolean[] pixOn = new boolean[N_SENSORS] ;
        for (int i=0 ; i<N_SENSORS ; i++) {
            int pixX = (int) (mSensX[i]*mPixPerIn*12.0 + mImagePixW/2) ;
            int pixY = (int) (mImagePixH-1 - mSensY[i]*mPixPerIn*12.0) ;
            pixX = Math.min((int)mImagePixW-1, Math.max(pixX, 0)) ;
            pixY = Math.min((int)mImagePixH-1, Math.max(pixY, 0)) ;
            int pixVal = mImage.getRGB(pixX, pixY) ;
            //System.out.printf("sensor %d at (%d,%d) = %x\n", i, pixX, pixY, pixVal);
            // detect stop color   
            if (isStopColor(pixVal)) {
            	Robot.logger.appendLog("LineFollow Braking");
                mState = LineFollowState.BRAKE ;
                break ;
            }
            // convert pixval to on/off for convenience
            pixVal = pixVal & 0x00ffffff ;
            pixOn[i] = false ;
            if (pixVal < PIX_THRESH)
                pixOn[i] = true ;                
        }
        //System.out.println(Arrays.toString(pixOn));    
        
        double leftAccel, rightAccel ;
        if (mState==LineFollowState.BRAKE) {
            leftAccel = -velFtPerSec*BRAKE_PER_VEL_IN_FTPS ;
            rightAccel = -velFtPerSec*BRAKE_PER_VEL_IN_FTPS ;
        }
        else {
            double pos = getPositionVal(pixOn) ;
            //System.out.println("pos = " + pos);
            double error = pos-SENSOR_POS_MIDINDEX ;

            // update acceleration controls based on position error
            double deriv = (error-mLastError)/dT ;
            mLastError = error ;
            mSumError += error*dT ;
            double speedAdj = KP*error + KI*mSumError + KD*deriv ;
            // System.out.println(speedAdj);
            
            leftAccel = mBaseAccel + speedAdj ;
            rightAccel = mBaseAccel - speedAdj ;
            // oldNonPidControl(pixOn);
        }

        // rail the accelerations to +/- 1
        leftAccel = Math.max(leftAccel, -1) ;
        leftAccel = Math.min(leftAccel, 1) ;
        rightAccel = Math.max(rightAccel, -1) ;
        rightAccel = Math.min(rightAccel, 1) ;
        
        MotorControlStruct motorControl = new MotorControlStruct() ;
        motorControl.left = leftAccel ;
        motorControl.right = rightAccel ;

        return motorControl ;
    }
        
    private void upDateSensorLocs (double x, double y, double absThetaDegCCW) {
        double theta = absThetaDegCCW*Math.PI/180.0 ;        
        double xOffset = -SENSOR_POS_MIDINDEX * mSensorSpacingFt ;
        for (int i=0 ; i<N_SENSORS ; i++) {
            mSensX[i] = x + SENSOR_STANDOFF_FT*Math.cos(theta)
                          + xOffset*Math.sin(theta) ;
            mSensY[i] = y + SENSOR_STANDOFF_FT*Math.sin(theta)
                          - xOffset*Math.cos(theta) ;
            xOffset += mSensorSpacingFt ;
        } 
    }
    
    private double getPositionVal(boolean[] pixOn) {
        boolean seeLine = false ;
        double sum = 0 ;
        double nOn = 0 ;
        for (int i=0 ; i<pixOn.length ; i++) {
            if (pixOn[i]) {
                seeLine = true ;
                sum += i ;
                nOn++ ;
            }
        }
        // if didn't see the line, return a val based on the last seen val
        if (!seeLine) {
            // if last seen to the left of center, return 0
            if ( mLastSensorVal < SENSOR_POS_MIDINDEX )
                return 0.0 ;
            else  // return max
                return ((double)N_SENSORS-1) ;
        }
        mLastSensorVal = sum / nOn ;
        return mLastSensorVal ;
    } 
    
    private boolean isStopColor(int pixVal) {
        boolean result = false ;
        
        // mask off the alpha and then the red
        int red = (pixVal & 0xff0000) >> 16 ;
        int green = (pixVal & 0x00ff00) >> 12 ;
        int blue = (pixVal & 0x0000ff) ;  
        // System.out.println("r="+red+" g="+green+" b="+blue);
        if (red>0x80 && green<0x80 && blue<0x80)
            result = true ;
        return result ;
    }    
    
    public LineFollowState getControllerState() {
        return mState ;
    }

    public boolean isFinished() {
        return (mState==LineFollowState.DONE) ;
    }
}
