import java.io.*;
import java.util.*;
import java.net.*;

public class Sender{


	public Sender(){
		try{
			//Creo il Socket
			//InetSocketAddress isa = new InetSocketAddress("localhost", 8080);
			Socket s = new Socket("localhost", 8080);
			//s.connect(isa);
			OutputStream os = s.getOutputStream();
			BufferedWriter bw = new BufferedWriter( new OutputStreamWriter( os ), 8 );
			Scanner sc = new Scanner(System.in);
			while(true){
				System.out.print("> ");
				String str = sc.nextLine();
				//Scrivo in OUT
				bw.write( str, 0, str.length() );
				bw.newLine();
				bw.flush();
			}
		}catch(Exception e){
			System.out.println("Errore: \n"+e.toString());
			System.exit(0);
		}
	}//Sender

}//Sender
		
			
		
