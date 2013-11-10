package net.cloud4service.ubernode;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.net.InetAddress;
import java.net.Socket;
import java.util.concurrent.Semaphore;

/**
 * Created by andrea on 11/5/13.
 */
public class DataSender implements PerformedByClock{

    private String ipAddress;
    private int port;
    private Console console;
    private OutputStream outputStream = null;
    private BufferedWriter bufferedWriter = null;
    private boolean isConnected = false;

    //Connection
    private Socket socket;

    //Concurrency
    private Semaphore dataSemaphore;

    //TEMP
    private String data;



    private DataSender(){} //Lock default constructor

    public DataSender(Console c, String ip, int p){
        ipAddress = ip;
        port = p;
        console = c;
        //
        dataSemaphore = new Semaphore(1);
    }//Sender


    public void connect(){
        //verify reachability
        try {
            long currentTime = System.currentTimeMillis();
            if( !InetAddress.getByName(ipAddress).isReachable(1000) ){
                console.printToConsole("Host "+ipAddress+" is unreachable! Test your connection.\n");
                return;
            }
            console.printToConsole("Host "+ipAddress+" is reachable: "+(System.currentTimeMillis()-currentTime)+"ms latency");
        } catch (IOException e) {
            console.printToConsole(e.toString());
            return;
        }
        //Host successfully reached
        try{
            //Creo il Socket
            socket = new Socket(ipAddress, port);
            console.printToConsole("Connecting to: "+ipAddress+":"+port);
            outputStream = socket.getOutputStream();
            bufferedWriter = new BufferedWriter( new OutputStreamWriter( outputStream ) );
            console.printToConsole("Connection established!\n");
            isConnected = true;
        }catch(Exception e){
            isConnected = false;
            if(socket == null){
                console.printToConsole("Connection to: "+ipAddress+":"+port+" failed!");
            }
            //Do nothing
            return;
        }
    }//connect

    public boolean isConnected(){
        return isConnected;
    }//isConnected

    public void appendData(String data){
        try {
            dataSemaphore.acquire();
            //=> Sezione critica
            this.data = data;
            //<= Sezione critica
            dataSemaphore.release();
        } catch (InterruptedException e) {
            dataSemaphore.release();
            //do nothing
        }
    }//appendData

    private void send(String str){
        if(bufferedWriter != null){
            try {
                bufferedWriter.write(str, 0, str.length());
                bufferedWriter.newLine();
                bufferedWriter.flush();
            } catch (IOException e) {
                //do nothing
                console.printToConsole("Connection lost!");
                isConnected = false;
                //close connection
                close();
                //null-ize
                bufferedWriter = null;
                socket = null;
                return;
            }
        }else{
            //Not connected
            //Do nothing
            return;
        }
    }//send

    @Override
    public void perform() {
        try {
            dataSemaphore.acquire();
            //=> Sezione critica
            if(data != null){
                send(data);
            }
            //<= Sezione critica
            dataSemaphore.release();
        } catch (InterruptedException e) {
            //do nothing
            dataSemaphore.release();
        }
    }//

    public void close(){
        //Close connection
        try{
            if(bufferedWriter != null){ bufferedWriter.close(); }
            if(outputStream != null){ outputStream.close(); }
            if(socket != null){ socket.close(); }
        }catch(Exception ex){/*do nothing*/}
        isConnected = false;
    }//close

}//Sender