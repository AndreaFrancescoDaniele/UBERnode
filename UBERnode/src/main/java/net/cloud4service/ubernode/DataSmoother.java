package net.cloud4service.ubernode;

import android.preference.Preference;

import java.math.BigDecimal;
import java.util.Arrays;

/**
 * Created by andrea on 6/11/13.
 */
public class DataSmoother {

    //Official data
    private static float[] data = {0, 0, 0};

    //Parametri di Smoothing
    //UpsetZ
    private static float upsetLimit = 4f;
    //MagneticPoint
    private static float radius = 2f;
    //Boundary
    private static float limit = 6.5f;
    //Normalize
    private static float upperValue = 10f;
    //Stepper
    private static float step = 0.5f;

    //Temp
    private static float normalize_max;
    private static float normalize_sign;


    private DataSmoother(){} //lock default constructor

    //public DataSmoother(Preference)


    //New received data
    private static float[] ndata;

    public static float[] smooth(float x, float y, float z){
        //Raw data
        ndata = new float[3]; ndata[0]=x; ndata[1]=y; ndata[2]=z;
        //Applying smoothing policies
        //First (upsetZ)
        upsetZ(z);
        //Others
        invertSignum();
        magneticPoint();
        boundary();
        normalize();
        stepper();
        //Update official data
        data = Arrays.copyOf(ndata, 3);
        //Return data
        return ndata;
    }//smooth

    private static void upsetZ(float z){
        if(z <= upsetLimit){
            //phone upside down
            ndata[0] = 0;
            ndata[1] = 0;
        }
    }//upsetZ

    private static void invertSignum(){
        ndata[0] = ndata[0] * (-1f);
        ndata[1] = ndata[1] * (-1f);
    }//invertSignum

    private static void magneticPoint(){
        if(Math.abs(ndata[0]) < radius){
            ndata[0] = 0;
        }
        if(Math.abs(ndata[1]) < radius){
            ndata[1] = 0;
        }
    }//magneticPoint

    private static void boundary(){
        if(Math.abs(ndata[0]) > limit){
            ndata[0] = limit*Math.signum(ndata[0]);
        }
        if(Math.abs(ndata[1]) > limit){
            ndata[1] = limit*Math.signum(ndata[1]);
        }
    }//boundary

    private static void normalize(){
        //max reachable value
        normalize_max = limit - radius;
        if( ndata[0] != 0 ){
            //sign
            normalize_sign = Math.signum( ndata[0] );
            //translate values
            ndata[0] = Math.abs( ndata[0] ) - radius;
            //normalize
            ndata[0] = normalize_sign * ( ndata[0] * ( upperValue / normalize_max ) );
        }
        if( ndata[1] != 0 ){
            //sign
            normalize_sign = Math.signum( ndata[1] );
            //translate values
            ndata[1] = Math.abs( ndata[1] ) - radius;
            //normalize
            ndata[1] = normalize_sign * ( ndata[1] * ( upperValue / normalize_max ) );
        }
    }//normalize

    private static void stepper(){
        ndata[0] = ((float) ( ((int)ndata[0]/step)*step ) );
        ndata[1] = ((float) ( ((int)ndata[1]/step)*step ) );
        ndata[2] = round( ndata[2], 2 );
    }//stepper




    //=================
    //Utility Methods
    private static float round(float d, int decimalPlace) {
        BigDecimal bd = new BigDecimal(Float.toString(d));
        bd = bd.setScale(decimalPlace, BigDecimal.ROUND_HALF_UP);
        return bd.floatValue();
    }//round

}//DataSmoother
