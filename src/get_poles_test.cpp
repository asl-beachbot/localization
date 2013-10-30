#include "ros/ros.h"
#include "std_msgs/String.h"
#include "laser_loc/scan_vector.h"
#include "laser_loc/scan_point.h"
#include <cmath>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "get_poles_test");
	ros::NodeHandle n;
	ros::Publisher pole_scan_pub = n.advertise<laser_loc::scan_vector>("pole_scan",1000);
	ros::Rate loop_rate(25);
	int count = 0;
	ROS_INFO("started Node\n");
	while(ros::ok()) 
	{
		//safe parameters for input generation
		double x=1;		
		double y=1;
		double theta=1;
		//get pose from parameters for dynamic visualization
		n.getParam("x",x);
		n.getParam("y",y);
		n.getParam("theta",theta);
		//pole coordinates
		double xp1 = 0;
		double yp1 = 0;
		double xp2 = 10;
		double yp2 = 0;
		//create scan points for poles
		laser_loc::scan_point p1;		
		laser_loc::scan_point p2;
		p1.distance = pow(pow((x-xp1),2)+pow((y-yp1),2),0.5);	//give distance
		p1.angle = atan2((y-yp1),(x-xp1))+3.1415927-theta;		//give angle
		p2.distance = pow(pow(x-xp2,2)+pow(y-yp2,2),0.5);
		p2.angle = atan2((y-yp2),(x-xp2))+3.1415927-theta;
		laser_loc::scan_vector poles;		//create set of scan points
		poles.scans.push_back(p1);
		poles.scans.push_back(p2);
		pole_scan_pub.publish(poles);		//publish scan points
		ros::spinOnce();
		loop_rate.sleep();
	}
}