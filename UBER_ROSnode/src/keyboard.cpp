#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define MAX_VEL 0.5;
#define MAX_YAW 1.0;

struct termios cooked, raw;

void loop(ros::NodeHandle n,ros::Publisher pub);

int main(int argc, char** argv)
{
    ros::init(argc,argv,"keyboard", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    ros::NodeHandle n;	
    ros::Publisher pub;    
    pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 30);   
    loop(n,pub);  
    return(0);
}

void loop(ros::NodeHandle n,ros::Publisher pub)
{
    char c;
    int kfd = 0;
    int speed = 0;
    int turn = 0;
    int flag=0;
    geometry_msgs::Twist cmd_vel;

    //Raw mode (??)
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Use WASD keys to move U.B.E.R.");
    
    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;

    while(1)
    {

        //polling
        int num;
        if ((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
	{
		//stop the robot
		if(flag){
			cmd_vel.linear.x = 0;
        		cmd_vel.angular.z = 0;
       	 		pub.publish(cmd_vel);
			flag=0;
		}else flag=1;
		continue;
        }
        switch(c)
        {
            case 0x77://W
                speed = 1;
                turn = 0;
                break;
            case 0x73://S
                speed = -1;
                turn = 0;
                break;
            case 0x61://A
                speed = 0;
                turn = 1;
                break;
            case 0x64://D
                speed = 0;
                turn = -1;
                break;
        }
        
        cmd_vel.linear.x = speed * MAX_VEL;
        cmd_vel.angular.z = turn * MAX_YAW;
        pub.publish(cmd_vel);
    }
}
