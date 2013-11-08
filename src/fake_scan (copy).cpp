#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

#define PI 3.141592653589

int main(int argc, char **argv) {
	ros::init(argc, argv, "fake_scan");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("scan",1000);
	ros::Rate loop_rate(25);

	srand (time(NULL));

	double angle1 = -45;
	double angle2 = 45;
	double dist1 = 7.071067812;
	double dist2 = 7.071067812;
	double angle_increment_deg = 0.25;
	double angle_increment_rad = angle_increment_deg/360*2*PI;
	double angle_min = -135.0;
	double angle_max = 135.0;
	double error = 0.08;	//[m]

	while(ros::ok()) {
		sensor_msgs::LaserScan scan;
		scan.header.seq = 1;
		scan.header.stamp = ros::Time::now();
		scan.header.frame_id = "laser_frame";
		scan.angle_min = angle_min/360*2*PI;
		scan.angle_max = angle_max/360*2*PI;
		scan.angle_increment = angle_increment_rad;

		for (int i = 0; i < (angle_max-angle_min)/angle_increment_deg; i++) {
			if(i == (int)(angle1-angle_min)/angle_increment_deg) {
				scan.ranges.push_back(dist1);
				//scan.ranges.push_back(dist1 + (rand() % 100) / (100/error) - error/2);	//with error
				scan.intensities.push_back(2000);
				//ROS_INFO("pushed %f at index %u", scan.ranges.back(), i);
			}
			if(i == (int)(angle1-angle_min)/angle_increment_deg+1) {
				scan.ranges.push_back(dist1);
				//scan.ranges.push_back(dist1 + (rand() % 100) / (100/error) - error/2);
				scan.intensities.push_back(2000);
				//ROS_INFO("pushed %f at index %u", scan.ranges.back(), i);
			}
			if(i == (int)(angle2-angle_min)/angle_increment_deg) {
				scan.ranges.push_back(dist2);
				//scan.ranges.push_back(dist2 + (rand() % 100) / (100/error) - error/2);
				scan.intensities.push_back(2000);
				//ROS_INFO("pushed %f at index %u", scan.ranges.back(), i);	
			}
			else {
				scan.ranges.push_back(0.0);
				scan.intensities.push_back(1);
			}
		}
		//ROS_INFO("scan has %lu entries", scan.ranges.size());
		pub.publish(scan);
		ros::spinOnce();
		loop_rate.sleep();
	}

}