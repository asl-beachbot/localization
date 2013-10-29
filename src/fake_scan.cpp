#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>

#define PI 3.141592653589

int main(int argc, char **argv) {
	ros::init(argc, argv, "fake_scan");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("scan",1000);
	ros::Rate loop_rate(25);

	double angle1 = 25.0;
	double angle2 = -52.0;
	double dist1 = 8.91667;
	double dist2 = 6.666667;
	double angle_increment_rad = 0.25/360*2*PI;
	double angle_increment_deg = 0.25;
	double angle_min = -135.0;
	double angle_max = 135.0;

	while(ros::ok()) {
		sensor_msgs::LaserScan scan;
		scan.header.seq = 1;
		scan.header.stamp = ros::Time::now();
		scan.header.frame_id = "laser_frame";
		scan.angle_min = -PI/2;
		scan.angle_max = PI/2;
		scan.angle_increment = PI/1080;

		for (int i = 0; i < (angle_max-angle_min)/angle_increment_deg; i++) {
			if(i == (int)(angle1-angle_min)/angle_increment_deg) {
				scan.ranges.push_back(dist1);
				scan.intensities.push_back(2000);
				ROS_INFO("pushed %f at index %u", scan.ranges.back(), i);
			}
			if(i == (int)(angle1-angle_min)/angle_increment_deg+1) {
				scan.ranges.push_back(dist1+0.1);
				scan.intensities.push_back(2000);
				ROS_INFO("pushed %f at index %u", scan.ranges.back(), i);
			}
			if(i == (int)(angle2-angle_min)/angle_increment_deg) {
				scan.ranges.push_back(dist2);
				scan.intensities.push_back(2000);
				ROS_INFO("pushed %f at index %u", scan.ranges.back(), i);	
			}
			else {
				scan.ranges.push_back(0.0);
				scan.intensities.push_back(1);
			}
		}
		ROS_INFO("scan has %lu entries", scan.ranges.size());
		pub.publish(scan);
		ros::spinOnce();
		loop_rate.sleep();
	}

}