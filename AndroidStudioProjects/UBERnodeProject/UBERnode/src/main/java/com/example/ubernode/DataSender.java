package com.example.ubernode;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.net.Socket;

/**
 * Created by andrea on 11/5/13.
 */
public class DataSender {

    private BufferedWriter bufferedWriter = null;


    private DataSender(){} //Lock default constructor

    public DataSender(String serverIp, int port){
        try{
            //Creo il Socket
            Socket s = new Socket(serverIp, port);
            OutputStream os = s.getOutputStream();
            bufferedWriter = new BufferedWriter( new OutputStreamWriter( os ) );
        }catch(Exception e){
            e.printStackTrace();
            System.exit(-1);
        }
    }//Sender

    public void send(String str){
        if(bufferedWriter != null){
            try {
                bufferedWriter.write(str, 0, 4);
                bufferedWriter.newLine();
                bufferedWriter.flush();
            } catch (IOException e) {
                e.printStackTrace();
                System.exit(-1);
            }
        }else{
            //Non Connesso
            //Non fare nulla
        }
    }//send

}//Sender