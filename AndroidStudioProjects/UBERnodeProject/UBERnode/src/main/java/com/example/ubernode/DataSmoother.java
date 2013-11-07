package com.example.ubernode;

import java.math.BigDecimal;
import java.util.Arrays;

/**
 * Created by andrea on 6/11/13.
 */
public class DataSmoother {

    //Official data
    private static float[] data = {0, 0, 0};

    //Parametri di Smoothing
    
    private static float accuracy = 1f;
    //UpsetZ
    private static float upsetLimit = 4f;
    //MagneticPoint
    private static float radius = 2f;
    //WeightSmoothing
    private static float previousWeight = 0.6f;
    private static float newWeight = 0.4f;
    //Boundary
    private static float limit = 6.5f;
    //Cropping
    private static int decimalNumbers = 2;
    //Normalize
    private static float upperValue = 10f;
    private static float lowerValue = 0f;

    //Temp
    private static float normalize_max;
    private static float normalize_sign;


    //New received data
    private static float[] ndata;

    public static float[] smooth(float x, float y, float z){
        //Raw data
        ndata = new float[3]; ndata[0]=x; ndata[1]=y; ndata[2]=z;
        //Applying smoothing policies
        //Precision
        //precisionNoiseAvoid();
        stepper();
        //First (upsetZ)
        upsetZ(z);
        //Others
        invertSignum();
        magneticPoint();
        //boundary();

        //normalize();
        cropping();
        //Last (weightSmoothing)
        //weightSmoothing();
        //Update official data
        data = Arrays.copyOf(ndata, 3);
        //data = ndata;
        //Return data
        return ndata;
    }//smooth

    private static void stepper(){
        float step = 0.5f;
        ndata[0] = ((float) ( ((int)ndata[0]/step)*step ) );
        ndata[1] = ((float) ( ((int)ndata[1]/step)*step ) );
        ndata[2] = ((float) ( ((int)ndata[2]/step)*step ) );
    }//

    /*
    private static void precisionNoiseAvoid(){
        if( Math.signum(data[0]) == Math.signum(ndata[0]) ){
            if( Math.abs( Math.abs(data[0]) - Math.abs(ndata[0]) ) < accuracy ){
                ndata[0] = data[0];
            }
        }
        if( Math.signum(data[1]) == Math.signum(ndata[1]) ){
            if( Math.abs( Math.abs(data[1]) - Math.abs(ndata[1]) ) < accuracy ){
                ndata[1] = data[1];
            }
        }
        if( Math.signum(data[2]) == Math.signum(ndata[2]) ){
            if( Math.abs( Math.abs(data[2]) - Math.abs(ndata[2]) ) < accuracy ){
                ndata[2] = data[2];
            }
        }

        //error = Math.abs( Math.abs(data[2]) - Math.abs(ndata[2]) );
        //error = Math.abs(Math.abs(data[2]) - Math.abs(ndata[2]));

        if( Math.abs( Math.abs(data[2]) - Math.abs(ndata[2]) ) < accuracy ){
            ndata[2] = data[2];
        }

    }//precisionNoiseAvoid
    */

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

    private static void cropping(){
        ndata[0] = round(ndata[0], decimalNumbers);
        ndata[1] = round(ndata[1], decimalNumbers);
        ndata[2] = round(ndata[2], decimalNumbers);
    }//cropping

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

    private static void weightSmoothing(){
        ndata[0] = data[0]*previousWeight + ndata[0]*newWeight;
        ndata[1] = data[1]*previousWeight + ndata[1]*newWeight;
    }//weightSmoothing




    //=================
    //Utility Methods
    private static float round(float d, int decimalPlace) {
        BigDecimal bd = new BigDecimal(Float.toString(d));
        bd = bd.setScale(decimalPlace, BigDecimal.ROUND_HALF_UP);
        return bd.floatValue();
    }//round

}//DataSmoother
