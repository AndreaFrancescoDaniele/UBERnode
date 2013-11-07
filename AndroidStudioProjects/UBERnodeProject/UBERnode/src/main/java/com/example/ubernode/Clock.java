package com.example.ubernode;


import android.app.Activity;
import android.view.View;

/**
 * Created by andrea on 11/5/13.
 */
public class Clock extends Thread{

    private final int clock = 1000; //100mS = 0.1S
    private MainActivity mainActivity = null;

    private Clock(){} //Lock default constructor

    public Clock(MainActivity ma){
        mainActivity = ma;
    }//Clock

    @Override
    public void run(){
        if(mainActivity != null){
            try{
                //Cyclically
                while(true){
                    //Wait for a time-step
                    Thread.sleep(clock);
                    //Do something

                }
            }catch (Exception e){
                //Close app
                //System.exit(0);
                //mainActivity.console.setText( e.toString()+'\n'+mainActivity.console.getText() );
            }
        }
    }//run

}//Clock
