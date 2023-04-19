#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "first_project/custom_odometry.h"
#include "first_project/reset_odom.h"
#include <tf/transform_broadcaster.h>

#include <sstream>





class odometry
{

geometry_msgs::Quaternion speed_angle;

std_msgs::Float64 m1;
std_msgs::Float64 m2;

	double v_x;
	double v_y;
	double w_center;
	double d= 2.8;// distance betwen axes
	
	double x;
	double y;
	double th;

	double x_b;
	double y_b;
	double th_b;

	double dt =0.1; 


private:
ros::NodeHandle n; 

ros::Subscriber sub;

ros::Publisher pubodom;

ros::ServiceServer reset_odom_service;

ros::Publisher pubcustom; 

ros::Publisher angle; // for debug 

ros::Publisher speed; // for debug 


ros::Timer timer1;
	
	
public:


	

  	odometry(){
 
    // debug area 
	angle = n.advertise<std_msgs::Float64>("angle", 1);
	speed= n.advertise<std_msgs::Float64>("speed", 1);
   //debug area

	// START DEFINITION ALL SERVICES
   	reset_odom_service = n.advertiseService("reset_odom",&odometry::reset, this);
	// END DEFINITION ALL SERVICES


	// START DEFINITION ALL SUBSCRIBE TOPICS
  	sub = n.subscribe("/speed_steer", 1, &odometry::callback, this);
	// END DEFINITION ALL SUBSCRIBE TOPICS


	// START DEFINITION ALL PUBLISH TOPICS
	pubodom = n.advertise<nav_msgs::Odometry>("/odometry", 1);
	pubcustom = n.advertise<first_project::custom_odometry>("/custom_odometry", 1);
	// END DEFINITION ALL PUBLISH TOPICS

	// START DEFINITION ALL TIMERS
	timer1 = n.createTimer(ros::Duration(dt), &odometry::callback1, this);
	// END DEFINITION ALL TIMERS
		
	// tf
	
	// START GET STATIC PARAMS
	n.getParam("/starting_x", x);
	n.getParam("/starting_y", y);
	n.getParam("/starting_th", th);
	// END GET STATIC PARAMS
	

}
void callback(const geometry_msgs::Quaternion::ConstPtr& msg){
speed_angle=*msg;

}

void callback1(const ros::TimerEvent& ev)
{
	//for debug TRY TO READ THE BAG FILE
	odometry_calc();
	m1.data=x;
	m2.data=y;
	speed.publish(m1);
	angle.publish(m2);
	
	tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Quaternion q;
    transform.setOrigin( tf::Vector3(x_b, y_b, 0) );
    q.setRPY(0, 0, th_b);
	transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "vehicle_centre"));
	
  	ROS_INFO("Callback 1 triggered");
	//for debug TRY TO READ THE BAG FILE
}

// START OF KINEMATIC EQUATIONS

double dx(float x1, float y1, float th1){
	return(speed_angle.x*cos(th1)); // speed of the back wheel
}

double dy(float x1, float y1, float th1){
	return (speed_angle.x*sin(th1)); // speed of the back wheel
}

double dth( float x1, float y1, float th1){
	return	(speed_angle.x*tan(speed_angle.y)/d ); // speed of the back wheel
}

// END OF KINEMATICS EQUATIONS


void rngkutta(float h){
	float k1=h* dx(x,y,th);
	float l1=h* dy(x,y,th);
	float m1=h* dth(x,y,th);

	float k2=h* dx(x+1/2*k1,y+1/2*l1,th+1/2*m1);
	float l2=h* dy(x+1/2*k1,y+1/2*l1,th+1/2*m1);
	float m2=h* dth(x+1/2*k1,y+1/2*l1,th+1/2*m1);

	float k3=h* dx(x+1/2*k2,y+1/2*l2,th+1/2*m2);
	float l3=h* dy(x+1/2*k2,y+1/2*l2,th+1/2*m2);
	float m3=h* dth(x+1/2*k2,y+1/2*l2,th+1/2*m2);

	float k4=h* dx(x+k3,y+l3,th+m3);
	float l4=h* dy(x+k3,y+l3,th+m3);
	float m4=h* dth(x+k3,y+l3,th+m3);

	x=x+(1/6)*(k1+2*k2+2*k3+k4);
	y=y+(1/6)*(l1+2*l2+2*l3+l4);
	th=th+(1/6)*(m1+2*m2+2*m3+m4);
}





void odometry_calc(){
	w_center=speed_angle.x*cos(speed_angle.y*tan(speed_angle.y)/d);
	v_y=w_center*d/2;
	v_x=w_center*d/tan(speed_angle.y);
	
	//HERE PUT THE ODEMETRY CALCULATION
	rngkutta(dt);
}

bool reset(first_project::reset_odom::Request  &req,
           first_project::reset_odom::Response &res)
{
    // resetting the odometry ...
	x=0;
	y=0;
	th=0;
	x_b=0;
	y_b=0;
	th_b=0;	
  res.resetted = true;   // reset is done
  return true;
}


};



int main(int argc, char **argv){

	ros::init(argc, argv, "odom_node");
	odometry node_core;
	ros::spin();
  	return 0;
}
