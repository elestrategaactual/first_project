#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "first_project/custom_odometry.h"

#include <sstream>

int main(int argc, char **argv){
    
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;

	//ros::Publisher chatter_pub = n.advertise<custom_messages::custom_odometry>("chatter", 1000);//aqui se coloca el lugar donde esta el custom mesage en este caso custom_messages

	ros::Rate loop_rate(10);

	int count = 0;
  
  
  	while (ros::ok()){
	    
	    //	static int i=0;
		//i=(i+1)%1000;
		//custom_messages::Num msg;
		//msg.num =i;
		//chatter_pub.publish (msg);

  	}
//hello
// hello 2
  	return 0;
}
