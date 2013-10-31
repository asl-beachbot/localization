#include <ros/ros.h>
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"

class localization {
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Publisher pubPose;
	ros::Publisher pubPole;


	void locate() {

	}

	void callback(const sensor_msgs::LaserScan &scan) {

	}



public:
	localization() {
		sub = n.subscribe("scan",1000, &localization::callback, this);
		pubPose = n.advertise<geometry_msgs::PoseStamped>("bot_pose",1000);
		pubPole = n.advertise<geometry_msgs::PointStamped>("pole_pos",1000);
	}







};

int main(int argc, char **argv) {
	ros::init(argc, argv, "get_poles");
	localization *loc = new localization;





}