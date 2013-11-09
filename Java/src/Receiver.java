import java.net.ServerSocket;
import java.net.Socket;
import java.util.*;
import java.io.*;


public class Receiver {

	private int port = 0;
	private ServerSocket ss;
	
	private ObjectOutputStream prova;
	
	@SuppressWarnings("unused")
	private Receiver(){}
	
	public Receiver(int port){
		this.port = port;
		System.out.println("Avvio del server in corso...");
		//Creo il serverSocket
		try{ 
			ss = new ServerSocket( this.port ); 
		}catch(IOException e){
			System.out.println("Avvio del Server non riuscito: \n"+e.toString());
			return;
		}
		//Avvio la Connessione
		System.out.println("Server in ascolto sulla porta: "+this.port+"\n");
		try{ 
			Socket s = ss.accept();
			//Info
			String client = s.getRemoteSocketAddress().toString();
			System.out.println("Richiesta di connessione dal client "+client+"...");
			InputStream is = s.getInputStream();
			BufferedReader br = new BufferedReader( new InputStreamReader( is ) );
			while(true){
				Thread.sleep(50);
				if( br.ready() ){
					System.out.println( br.readLine() );
				}
			}
		}catch(Exception e){
			System.out.println("Errore di connessione... \n "+e.toString());
			System.exit(0);
		}
	}//Receiver

}//Receiver








