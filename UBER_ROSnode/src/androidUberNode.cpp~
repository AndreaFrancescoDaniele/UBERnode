#include "PracticalSocket.h"  // For Socket, ServerSocket, and SocketException
#include <iostream>
#include <ros/ros.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>

#define MAX_VEL 1.0;
//#define MAX_VEL 0.5;
#define MAX_YAW 1.0;

using namespace std;

//Parameters
char c;
int speed = 0;
int turn = 0;
int flag=0;
double normalize_upperValue = 10;
geometry_msgs::Twist cmd_vel;
const unsigned int bufferSize = 32;
//Prototypes
void HandleTCPClient(TCPSocket *sock, ros::NodeHandle n, ros::Publisher pub); // TCP client handling function
string subString(char *echoBuffer, int length);
void loop(ros::NodeHandle n,ros::Publisher pub);
void extractValues(double* array, string str);
void stopRobot(ros::Publisher pub);


int main(int argc, char** argv){
	//Configuration: ROS-side
    ros::init(argc,argv,"androidUberNode", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::Publisher pub;    
    pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 30);
	//Configuration: Network-side
  	unsigned short serverPort = 8080;
	//create a Socket and wait for a client
  	try {
    	TCPServerSocket servSock(serverPort);
    	while(1){
      		HandleTCPClient(servSock.accept(), n, pub);
		}
	} catch (SocketException &e) {
		cerr << e.what() << endl;
		return -1;
	}
	//Call the loop function
    //loop(n,pub);
    return(0);
}//main


// TCP client handling function
void HandleTCPClient(TCPSocket *sock, ros::NodeHandle n, ros::Publisher pub) {
	cout << endl;
	cout << "|===============================|" << endl;
	cout << "| Android uberNode connected:" << endl;
	cout << "| IP: ";
  	try {
		cout << sock->getForeignAddress() << endl;
	} catch (SocketException e) {
    	cout << "ND" << endl;
	}
	cout << "| Port: ";
	try {
		cout << sock->getForeignPort() << endl;
	} catch (SocketException e) {
		cout << "ND" << endl;
	}
	cout << "|===============================|" << endl << endl;
	//Receive commands
	char echoBuffer[bufferSize];
	int recvMsgSize;
	//Receive a command
	while ( (recvMsgSize = sock->recv(echoBuffer, bufferSize)) > 0) { // Zero means end of transmission
		string message = subString(echoBuffer, recvMsgSize);
		//extract command from message
		if( message != "null" ){
			//valid Message: extract values from message
			double values[2] = { 0 };
			extractValues( values, message );
			//Landscape
			turn = values[1]; //roll
			speed = values[0]; //pitch
			/**/
			/*Portrait
			turn = -1*values[0]; //roll
			speed = values[1]; //pitch
			*/
		}else{
			//invalid Message: stop the robot
			speed = 0;
        	turn = 0;
		}
		cmd_vel.linear.x = (speed*(1/normalize_upperValue)) * MAX_VEL;
        cmd_vel.angular.z = (turn*(1/normalize_upperValue)) * MAX_YAW;
        pub.publish(cmd_vel);
	}
	delete sock;
}//HandleTCPClient


string subString(char *echoBuffer, int length){
	//validity control
	int stickSigns = 0;
	int dollarSigns = 0;
	int dots = 0;
	int pos = -1;
	for(int i=0; i<length; i++){
		switch(echoBuffer[i]){
			case '\n':
				pos = i;
				break;
			case '$':
				dollarSigns++;
				break;
			case '|':
				stickSigns++;
				break;
			case '.':
				dots++;
				break;
			default:
				break;
		}
	}
	if(stickSigns != 1 || dollarSigns != 2 || dots != 2){
		//invalid command
		return "null";
	}
	if(pos != -1){
		return string(echoBuffer, pos);
	}
	return "null";
}//subString


void extractValues(double* array, string str){
	//find delimiters
	int d1 = str.find_first_of('$');
	int d2 = str.find_first_of('$', d1+1);
	int stick = str.find('|');
	//Roll
	double roll = atof( str.substr(d1+1, stick-d1-1).c_str() );
	if(roll > normalize_upperValue){ 
		roll = normalize_upperValue; 
	}
	array[0] = roll;
	//Pitch
	double pitch = atof( str.substr(stick+1, d2-stick-1).c_str() );
	if(pitch > normalize_upperValue){ 
		pitch = normalize_upperValue; 
	}
	array[1] = pitch;
}//extractValues


void stopRobot(ros::Publisher pub){
	cmd_vel.linear.x = 0;
	cmd_vel.angular.z = 0;
	pub.publish(cmd_vel);
}//stopRobot
