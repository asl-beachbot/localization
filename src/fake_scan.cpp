#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

#define PI 3.141592653589

void correctAngle(double& angle) {
    while(angle > PI) angle -= 2*PI;
    while(angle < -PI) angle += 2*PI;
 }

int main(int argc, char **argv) {
	ros::init(argc, argv, "fake_scan");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("scan",1000);
	ros::Rate loop_rate(25);

	srand (time(NULL));

	double x=5;		
	double y=5;
	double theta=-PI/4;
	double xp1 = 0;
	double yp1 = 0;
	double xp2 = 10;
	double yp2 = 0;
	double xp3 = 10;
	double yp3 = 10;

	double angle1 = atan2((y-yp1),(x-xp1))+3.1415927-theta;
	double angle2 = atan2((y-yp2),(x-xp2))+3.1415927-theta;
	double angle3 = atan2((y-yp3),(x-xp3))+3.1415927-theta;
	correctAngle(angle1);
	correctAngle(angle2);
	correctAngle(angle3);
	//ROS_INFO("angle1 %f", angle1);
	//ROS_INFO("angle2 %f", angle2);
	double dist1 = pow(pow(x-xp1,2)+pow(y-yp1,2),0.5);
	double dist2 = pow(pow(x-xp2,2)+pow(y-yp2,2),0.5);
	double dist3 = pow(pow(x-xp3,2)+pow(y-yp3,2),0.5);

	//double angle_increment_deg = 0.25;
	double angle_increment_rad = 0.25/360*2*PI;
	double angle_min = -3.0/4*PI;
	double angle_max = 3.0/4*PI;
	double error = 0.08;	//[m]

	while(ros::ok()) {
		sensor_msgs::LaserScan scan;
		scan.header.seq = 1;
		scan.header.stamp = ros::Time::now();
		scan.header.frame_id = "laser_frame";
		scan.angle_min = angle_min;
		scan.angle_max = angle_max;
		scan.angle_increment = angle_increment_rad;

		for (int i = 0; i < (angle_max-angle_min)/angle_increment_rad; i++) {
			if(i == (int)((angle1-angle_min)/angle_increment_rad)) {
				//scan.ranges.push_back(dist1);
				scan.ranges.push_back(dist1 + (rand() % 100) / (100/error) - error/2);	//with error
				scan.intensities.push_back(2000);
				//ROS_INFO("pushed %f at index %u", scan.ranges.back(), i);
			}
			if(i == (int)((angle1-angle_min)/angle_increment_rad)+1) {
				//scan.ranges.push_back(dist1);
				scan.ranges.push_back(dist1 + (rand() % 100) / (100/error) - error/2);
				scan.intensities.push_back(2000);
				//ROS_INFO("pushed %f at index %u", scan.ranges.back(), i);
			}
			if(i == (int)((angle2-angle_min)/angle_increment_rad)) {
				//scan.ranges.push_back(dist2);
				scan.ranges.push_back(dist2 + (rand() % 100) / (100/error) - error/2);
				scan.intensities.push_back(2000);
				//ROS_INFO("pushed %f at index %u", scan.ranges.back(), i);	
			}
			if(i == (int)((angle3-angle_min)/angle_increment_rad)) {
				//scan.ranges.push_back(dist2);
				scan.ranges.push_back(dist3 + (rand() % 100) / (100/error) - error/2);
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