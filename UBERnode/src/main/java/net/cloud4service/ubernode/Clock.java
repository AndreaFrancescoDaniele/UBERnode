package net.cloud4service.ubernode;


import java.util.LinkedList;

/**
 * Created by andrea on 11/5/13.
 */
public class Clock extends Thread{

    private int clock = 1000; //100mS = 0.1S
    private LinkedList<PerformedByClock> list = new LinkedList<PerformedByClock>();

    private Clock(){} //Lock default constructor

    public Clock(int clockTimeMillis){
        clock = clockTimeMillis;
    }//Clock

    public void setClockDuration(int clockTimeMillis){
        clock = clockTimeMillis;
    }//setClockDuration

    public void addPerformer(PerformedByClock obj){
        if(!list.contains(obj)){
            list.add(obj);
        }
    }//addPerformer

    public void clearPerformersList(){
        list.clear();
    }//clearPerformersList

    @Override
    public void run() {
        try {
            //Cyclically
            while (true) {
                //Wait for a time-step
                Thread.sleep(clock);
                //Do something
                for(PerformedByClock pbc:list){
                    if(pbc != null){
                        pbc.perform();
                    }
                }
            }
        } catch (Exception e) {
            //Do nothing
            return;
        }
    }//run

}//Clock
