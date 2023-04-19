#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "first_project/Odom.h"
#include <tf/transform_broadcaster.h>

#include "first_project/reset_odom.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <sstream>

double x_b ;
double y_b ;
double th_b ;

bool reset(first_project::reset_odom::Request  &req,
           first_project::reset_odom::Response &res)
{
    // resetting the odometry 
    x_b = 0 ;
    y_b = 0 ;
    th_b = 0 ;
    res.resetted = true;   // reset is done
    return true;
}

class pub_sub
{
geometry_msgs::Quaternion speed_angle;
nav_msgs::Odometry odom;
first_project::Odom odom_cust_msg ;

std_msgs::Float64 m1;
std_msgs::Float64 m2;
std_msgs::Float64 m3;
std::string t_str ;

double Ts ;     // 0.2

double v_x;     //robot frame
double v_y;
double vx_b;    //base frame
double vy_b;
double w;
double d = 2.8 ;

double dt =0.1;

double time;




private:
ros::NodeHandle n;
ros::ServiceServer service = n.advertiseService("reset_odom", reset);

ros::Subscriber sub;
ros::Publisher odom_node;
ros::Publisher custom_odometry;
ros::Timer timer1;



tf::TransformBroadcaster br;

public:
pub_sub(){
    sub = n.subscribe("/speed_steer", 1, &pub_sub::callback, this);
    odom_node = n.advertise<nav_msgs::Odometry>("/odometry", 1);
    custom_odometry = n.advertise<first_project::Odom>("/custom_odometry", 1);
    //timer1 = n.createTimer(ros::Duration(0.01), &pub_sub::callback1, this);
    
    n.getParam("/starting_x", x_b);
    n.getParam("/starting_y", y_b);
    n.getParam("/starting_th", th_b);  
     
}



void callback(const geometry_msgs::Quaternion::ConstPtr& msg){
    speed_angle =* msg ;
    odometrycalc();

    // I publish immediatly after a new message from speed_steer
    callback1();
    
}

void callback1()    //const ros::TimerEvent&
{
    //PUBLISHING
    // odmetry
    odom.pose.pose.position.x = x_b;
    odom.pose.pose.position.y = y_b;
    odom.pose.pose.orientation.x = x_b ;
    odom.pose.pose.orientation.y = y_b ;
    odom.pose.pose.orientation.w = th_b ;
    odom_node.publish(odom);
    // custum_odometry
    odom_cust_msg.x = x_b ;
    odom_cust_msg.y = y_b ;
    odom_cust_msg.th = th_b ;
    odom_cust_msg.timestamp = t_str;
    custom_odometry.publish(odom_cust_msg);
    // tf
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x_b, y_b, 0) );
    tf::Quaternion q;
    q.setRPY(0, 0, th_b);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "vehicle_centre"));

    
    ROS_INFO("Ts=%f",Ts);
}

void odometrycalc(){
    
    
    // Ts = 1/50 more or less from rosbag hz of the publication rate
    Ts = ros::Time::now().toSec() - time ;

    // EULERO INTEGRATION
    /*
    if(speed_angle.y != 0){
        w=speed_angle.x*tan(speed_angle.y)/d;
	    v_y=w*d/2;
	    v_x=w*d/tan(speed_angle.y);
    }
    else{
        w = 0;
    }
    
    th_b = th_b + w * Ts ;
    
    vx_b = (v_x * cos(th_b)) - (v_y * sin(th_b));
    vy_b = (v_x * sin(th_b)) + (v_y * cos(th_b));

    x_b = x_b + vx_b * Ts ;
    y_b = y_b + vy_b * Ts ;
    */
    // EULERO INTEGRATION

    rngkutta(dt);   
    time = ros::Time::now().toSec();    // time type to double
    t_str = std::to_string(time) ;      //double to string

    }

    //RUNGE-KUTTA
    
double dth(double x1, double y1, double th1){
	//return	(speed_angle.x*tan(speed_angle.y)/d ); // speed of the back wheel
	
	return(2*speed_angle.x*sin(speed_angle.y)/(d*(1+cos(speed_angle.y))));
}

double dx(double x1, double y1, double th1){
	
	//return(speed_angle.x*cos(th1)); // speed of the back wheel
	return((2*speed_angle.x*cos(speed_angle.y)/(1+cos(speed_angle.y)))*cos(th1)+(speed_angle.x*sin(speed_angle.y)/(1+cos(speed_angle.y)))*sin(th1));
}

double dy( double x1, double y1, double th1){
	//return (speed_angle.x*sin(th1)); // speed of the back wheel
	return((2*speed_angle.x*cos(speed_angle.y)/(1+cos(speed_angle.y)))*sin(th1)+(speed_angle.x*sin(speed_angle.y)/(1+cos(speed_angle.y)))*cos(th1));
}

void rngkutta(double h){
	double k1=h* dx(x_b,y_b,th_b);
	double l1=h* dy(x_b,y_b,th_b);
	double m1=h* dth(x_b,y_b,th_b);

	double k2=h* dx(x_b+0.5*k1,y_b+0.5*l1,th_b+0.5*m1);
	double l2=h* dy(x_b+0.5*k1,y_b+0.5*l1,th_b+0.5*m1);
	double m2=h* dth(x_b+0.5*k1,y_b+0.5*l1,th_b+0.5*m1);

	double k3=h* dx(x_b+0.5*k2,y_b+0.5*l2,th_b+0.5*m2);
	double l3=h* dy(x_b+0.5*k2,y_b+0.5*l2,th_b+0.5*m2);
	double m3=h* dth(x_b+0.5*k2,y_b+0.5*l2,th_b+0.5*m2);

	double k4=h* dx(x_b+k3,y_b+l3,th_b+m3);
	double l4=h* dy(x_b+k3,y_b+l3,th_b+m3);
	double m4=h* dth(x_b+k3,y_b+l3,th_b+m3);

	x_b=x_b+(1.0/6)*(k1+2.0*k2+2.0*k3+k4);
	y_b=y_b+(1.0/6)*(l1+2.0*l2+2.0*l3+l4);
	th_b=th_b+(1.0/6)*(m1+2.0*m2+2.0*m3+m4);


    ROS_INFO("x_b=%f",x_b);
    ROS_INFO("y_b=%f",y_b);
    ROS_INFO("th_b=%f",th_b);
}
    
    //RUNGE-KUTTA



};

int main(int argc, char **argv){
    ros::init(argc, argv, "odom_node") ;
    pub_sub my_pub_sub;
    ros::spin();
    return 0 ;
} 




