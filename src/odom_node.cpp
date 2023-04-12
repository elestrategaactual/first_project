#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "first_project/custom_odometry.h"

#include <sstream>

class pub_sub
{

geometry_msgs::Quaternion speed_angle;

std_msgs::Float64 m1;
std_msgs::Float64 m2;
    double x;
	double y;
	double th;
	double v_x;
	double v_y;
	double w_center;

	double x_b;
	double y_b;
	double th_b;
private:
ros::NodeHandle n; 

ros::Subscriber sub;

ros::Publisher pubodom;

ros::Publisher pubcustom; 

ros::Publisher angle; // for debug 

ros::Publisher speed; // for debug 


ros::Timer timer1;
	
	
public:

	

  	pub_sub(){
 
    // debug area 
	angle = n.advertise<std_msgs::Float64>("angle", 1);
	speed= n.advertise<std_msgs::Float64>("speed", 1);
   //debug area

  	sub = n.subscribe("/speed_steer", 1, &pub_sub::callback, this);
	pubodom = n.advertise<nav_msgs::Odometry>("/odometry", 1);
	pubcustom = n.advertise<first_project::custom_odometry>("/custom_odometry", 1);
	timer1 = n.createTimer(ros::Duration(0.1), &pub_sub::callback1, this);

	n.getParam("/starting_x", x);
	n.getParam("/starting_y", y);
	n.getParam("/starting_th", th);
	

}
void callback(const geometry_msgs::Quaternion::ConstPtr& msg){
speed_angle=*msg;

}

void callback1(const ros::TimerEvent& ev)
{
	//for debug TRY TO READ THE BAG FILE
	m1.data=speed_angle.x;
	m2.data=speed_angle.y;
	speed.publish(m1);
	angle.publish(m2);
  	ROS_INFO("Callback 1 triggered");
	//for debug TRY TO READ THE BAG FILE
}

void publish(){
	m1.data=speed_angle.x;
	m2.data=speed_angle.y;
	speed.publish(m1);
	angle.publish(m2);
}





};


int main(int argc, char **argv){

	ros::init(argc, argv, "odom_node");
	pub_sub my_pub_sub;
	ros::spin();
  	return 0;
}
